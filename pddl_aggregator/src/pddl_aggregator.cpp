#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <fstream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace robot_planner {

using json = nlohmann::json;

struct EntityInfo
{
  std::string name;
  std::string location;
};

struct PddlSections
{
  std::string problem_name;
  std::string domain_name;
  std::string objects_block;
  std::string init_block;
  std::string goal_block;
};

static std::string readFile(const std::string &path)
{
  if (path.empty()) {
    return {};
  }
  std::ifstream in(path);
  if (!in) {
    return {};
  }
  std::ostringstream oss;
  oss << in.rdbuf();
  return oss.str();
}

static std::optional<std::string> matchPddlToken(const std::string &src, const std::string &pattern)
{
  try {
    std::regex re(pattern, std::regex::icase);
    std::smatch m;
    if (std::regex_search(src, m, re) && m.size() > 1) {
      return m[1].str();
    }
  } catch (const std::exception &) {
  }
  return std::nullopt;
}

static std::string extractSection(const std::string &src, const std::string &keyword)
{
  const auto start = src.find(keyword);
  if (start == std::string::npos) {
    return {};
  }
  int depth = 0;
  bool started = false;
  for (size_t i = start; i < src.size(); ++i) {
    const char c = src[i];
    if (c == '(') {
      depth++;
      started = true;
    } else if (c == ')') {
      depth--;
      if (started && depth == 0) {
        return src.substr(start, i - start + 1);
      }
    }
  }
  return {};
}

static std::string insertBeforeClosingParen(const std::string &block, const std::string &addition)
{
  if (block.empty() || addition.empty()) {
    return block;
  }
  const auto pos = block.rfind(')');
  if (pos == std::string::npos) {
    return block + "\n" + addition;
  }
  std::string out = block;
  out.insert(pos, addition);
  return out;
}

static std::vector<EntityInfo> parseEntitiesJson(const std::string &payload, rclcpp::Logger logger)
{
  std::vector<EntityInfo> out;
  if (payload.empty()) {
    return out;
  }
  try {
    auto doc = json::parse(payload);
    if (!doc.is_array()) {
      RCLCPP_WARN(logger, "Expected JSON array for entities, got %s", doc.type_name());
      return out;
    }
    for (const auto &entry : doc) {
      if (!entry.is_object()) {
        continue;
      }
      const auto name_it = entry.find("name");
      const auto loc_it = entry.find("location");
      if (name_it == entry.end() || loc_it == entry.end()) {
        continue;
      }
      if (!name_it->is_string() || !loc_it->is_string()) {
        continue;
      }
      EntityInfo info;
      info.name = name_it->get<std::string>();
      info.location = loc_it->get<std::string>();
      if (!info.name.empty() && !info.location.empty()) {
        out.push_back(std::move(info));
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger, "Failed to parse entities JSON: %s", e.what());
  }
  return out;
}

static PddlSections parseMapPddl(const std::string &map_pddl)
{
  PddlSections sections;
  sections.problem_name = matchPddlToken(map_pddl, R"(\(define\s*\(problem\s+([^\s\)]+)\))")
                             .value_or("");
  sections.domain_name = matchPddlToken(map_pddl, R"(\(:domain\s+([^\s\)]+)\))")
                             .value_or("");
  sections.objects_block = extractSection(map_pddl, "(:objects");
  sections.init_block = extractSection(map_pddl, "(:init");
  sections.goal_block = extractSection(map_pddl, "(:goal");
  return sections;
}

class PddlProblemAggregator : public rclcpp::Node
{
public:
  PddlProblemAggregator()
  : Node("pddl_aggregator")
  {
    map_pddl_topic_ = declare_parameter<std::string>("map_pddl_topic", "/pddl/map");
    people_topic_ = declare_parameter<std::string>("people_topic", "");
    objects_topic_ = declare_parameter<std::string>("objects_topic", "");
    robots_topic_ = declare_parameter<std::string>("robots_topic", "");
    domain_file_ = declare_parameter<std::string>("domain_file", "");
    problem_output_file_ = declare_parameter<std::string>("problem_output_file", "/tmp/problem.pddl");
    publish_topic_ = declare_parameter<std::string>("problem_pddl_topic", "/pddl/problem");
    problem_name_override_ = declare_parameter<std::string>("problem_name", "");
    domain_name_override_ = declare_parameter<std::string>("domain_name", "");
    people_predicate_ = declare_parameter<std::string>("people_predicate", "person_at");
    objects_predicate_ = declare_parameter<std::string>("objects_predicate", "");
    robots_predicate_ = declare_parameter<std::string>("robots_predicate", "at");

    if (!domain_file_.empty() && domain_file_.front() != '/') {
      try {
        const auto share_dir = ament_index_cpp::get_package_share_directory("robot_planner");
        domain_file_ = share_dir + "/" + domain_file_;
      } catch (const std::exception &) {
      }
    }

    map_pddl_sub_ = create_subscription<std_msgs::msg::String>(
      map_pddl_topic_, 10, std::bind(&PddlProblemAggregator::mapPddlCallback, this, std::placeholders::_1));

    if (!people_topic_.empty()) {
      people_sub_ = create_subscription<std_msgs::msg::String>(
        people_topic_, 10, std::bind(&PddlProblemAggregator::peopleCallback, this, std::placeholders::_1));
    }
    if (!objects_topic_.empty()) {
      objects_sub_ = create_subscription<std_msgs::msg::String>(
        objects_topic_, 10, std::bind(&PddlProblemAggregator::objectsCallback, this, std::placeholders::_1));
    }
    if (!robots_topic_.empty()) {
      robots_sub_ = create_subscription<std_msgs::msg::String>(
        robots_topic_, 10, std::bind(&PddlProblemAggregator::robotsCallback, this, std::placeholders::_1));
    }

    if (!publish_topic_.empty()) {
      problem_pub_ = create_publisher<std_msgs::msg::String>(publish_topic_, 10);
    }

    get_domain_srv_ = create_service<std_srvs::srv::Trigger>(
      "get_domain_pddl", std::bind(&PddlProblemAggregator::handleGetDomain, this,
                                   std::placeholders::_1, std::placeholders::_2));
    get_problem_srv_ = create_service<std_srvs::srv::Trigger>(
      "get_problem_pddl", std::bind(&PddlProblemAggregator::handleGetProblem, this,
                                    std::placeholders::_1, std::placeholders::_2));
    get_domain_alias_srv_ = create_service<std_srvs::srv::Trigger>(
      "/pddl/get_domain", std::bind(&PddlProblemAggregator::handleGetDomain, this,
                                    std::placeholders::_1, std::placeholders::_2));
    get_problem_alias_srv_ = create_service<std_srvs::srv::Trigger>(
      "/pddl/get_problem", std::bind(&PddlProblemAggregator::handleGetProblem, this,
                                     std::placeholders::_1, std::placeholders::_2));
  }

private:
  void mapPddlCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    map_pddl_ = msg->data;
    refreshProblem();
  }

  void peopleCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    people_ = parseEntitiesJson(msg->data, get_logger());
    refreshProblem();
  }

  void objectsCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    objects_ = parseEntitiesJson(msg->data, get_logger());
    refreshProblem();
  }

  void robotsCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    robots_ = parseEntitiesJson(msg->data, get_logger());
    refreshProblem();
  }

  void handleGetDomain(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string domain_text = readFile(domain_file_);
    if (domain_text.empty()) {
      response->success = false;
      response->message = "Domain file is empty or not found.";
      return;
    }
    response->success = true;
    response->message = domain_text;
  }

  void handleGetProblem(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (problem_pddl_.empty()) {
      response->success = false;
      response->message = "Problem PDDL not available yet.";
      return;
    }
    response->success = true;
    response->message = problem_pddl_;
  }

  std::string resolveDomainName(const PddlSections &sections) const
  {
    if (!domain_name_override_.empty()) {
      return domain_name_override_;
    }
    if (!sections.domain_name.empty()) {
      return sections.domain_name;
    }
    const std::string domain_text = readFile(domain_file_);
    return matchPddlToken(domain_text, R"(\(define\s*\(domain\s+([^\s\)]+)\))")
      .value_or("domain");
  }

  std::string resolveProblemName(const PddlSections &sections) const
  {
    if (!problem_name_override_.empty()) {
      return problem_name_override_;
    }
    if (!sections.problem_name.empty()) {
      return sections.problem_name;
    }
    return "generated_problem";
  }

  std::string buildObjectsBlock(const PddlSections &sections) const
  {
    std::string objects_block = sections.objects_block;
    if (objects_block.empty()) {
      objects_block = "  (:objects\n  )";
    }
    std::ostringstream additions;
    std::unordered_set<std::string> emitted;

    auto add_entity = [&](const std::string &name, const std::string &type) {
      if (name.empty() || type.empty()) {
        return;
      }
      if (!emitted.insert(name).second) {
        return;
      }
      additions << "    " << name << " - " << type << "\n";
    };

    for (const auto &person : people_) {
      add_entity(person.name, "person");
    }
    for (const auto &obj : objects_) {
      add_entity(obj.name, "object");
    }
    for (const auto &robot : robots_) {
      add_entity(robot.name, "robot");
    }

    return insertBeforeClosingParen(objects_block, additions.str());
  }

  std::string buildInitBlock(const PddlSections &sections) const
  {
    std::string init_block = sections.init_block;
    if (init_block.empty()) {
      init_block = "  (:init\n  )";
    }
    std::ostringstream additions;

    auto add_predicate = [&](const std::string &predicate, const EntityInfo &info) {
      if (predicate.empty() || info.name.empty() || info.location.empty()) {
        return;
      }
      additions << "    (" << predicate << " " << info.name << " " << info.location << ")\n";
    };

    for (const auto &person : people_) {
      add_predicate(people_predicate_, person);
    }
    for (const auto &obj : objects_) {
      add_predicate(objects_predicate_, obj);
    }
    for (const auto &robot : robots_) {
      add_predicate(robots_predicate_, robot);
    }

    return insertBeforeClosingParen(init_block, additions.str());
  }

  std::string buildGoalBlock(const PddlSections &sections) const
  {
    if (!sections.goal_block.empty()) {
      return sections.goal_block;
    }
    return "  (:goal (and))";
  }

  void refreshProblem()
  {
    PddlSections sections = parseMapPddl(map_pddl_);
    const std::string domain_name = resolveDomainName(sections);
    const std::string problem_name = resolveProblemName(sections);

    std::ostringstream pddl;
    pddl << "(define (problem " << problem_name << ")\n"
         << "  (:domain " << domain_name << ")\n"
         << buildObjectsBlock(sections) << "\n"
         << buildInitBlock(sections) << "\n"
         << buildGoalBlock(sections) << "\n"
         << ")\n";

    problem_pddl_ = pddl.str();

    if (!problem_output_file_.empty()) {
      std::ofstream out(problem_output_file_);
      if (out) {
        out << problem_pddl_;
      }
    }

    if (problem_pub_) {
      std_msgs::msg::String msg;
      msg.data = problem_pddl_;
      problem_pub_->publish(msg);
    }
  }

  std::string map_pddl_topic_;
  std::string people_topic_;
  std::string objects_topic_;
  std::string robots_topic_;
  std::string domain_file_;
  std::string problem_output_file_;
  std::string publish_topic_;
  std::string problem_name_override_;
  std::string domain_name_override_;
  std::string people_predicate_;
  std::string objects_predicate_;
  std::string robots_predicate_;

  std::string map_pddl_;
  std::string problem_pddl_;
  std::vector<EntityInfo> people_;
  std::vector<EntityInfo> objects_;
  std::vector<EntityInfo> robots_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_pddl_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr people_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr objects_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robots_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr problem_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_domain_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_problem_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_domain_alias_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_problem_alias_srv_;
};

}  // namespace robot_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_planner::PddlProblemAggregator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
