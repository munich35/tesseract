/**
 * @file Environment.h
 * @brief Tesseract Environment.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_ENVIRONMENT_ENVIRONMENT_H
#define TESSERACT_ENVIRONMENT_ENVIRONMENT_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cstdint>
#include <functional>
#include <vector>
#include <string>
#include <chrono>
#include <set>
#include <utility>
#include <memory>
#include <shared_mutex>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/fwd.h>
#include <tesseract/geometry/fwd.h>
#include <tesseract/scene_graph/fwd.h>
#include <tesseract/state_solver/fwd.h>
#include <tesseract/srdf/fwd.h>
#include <tesseract/kinematics/fwd.h>
#include <tesseract/collision/fwd.h>
#include <tesseract/environment/fwd.h>

#include <filesystem>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/any_poly.h>
#include <tesseract/common/contact_allowed_validator.h>

namespace tesseract::environment
{
/**
 * @brief Function signature for adding additional callbacks for looking up TCP information
 *
 * The function should throw and exception if not located
 */
using FindTCPOffsetCallbackFn = std::function<Eigen::Isometry3d(const tesseract::common::ManipulatorInfo&)>;

using EventCallbackFn = std::function<void(const Event& event)>;

class Environment;
template <class Archive>
void serialize(Archive& ar, Environment& obj);

class EnvironmentContactAllowedValidator;
template <class Archive>
void serialize(Archive& ar, EnvironmentContactAllowedValidator& obj);

class EnvironmentContactAllowedValidator : public tesseract::common::ContactAllowedValidator
{
public:
  EnvironmentContactAllowedValidator() = default;  // Required for serialization
  EnvironmentContactAllowedValidator(std::shared_ptr<const tesseract::scene_graph::SceneGraph> scene_graph);

  bool operator()(const std::string& link_name1, const std::string& link_name2) const override;

protected:
  std::shared_ptr<const tesseract::scene_graph::SceneGraph> scene_graph_;

  template <class Archive>
  friend void ::tesseract::environment::serialize(Archive& ar, EnvironmentContactAllowedValidator& obj);
};

/**
 * @brief T-398 §7a: result of Environment::getChangesSince() - the commands applied since a
 * client-supplied external revision, or a refusal with the current external revision as a resync
 * hint if the client is too far ahead or too far behind (below the floor) to be answered.
 */
struct EnvironmentChanges
{
  bool success{ false };
  std::int64_t external_revision{ 0 };
  std::string id;
  std::vector<std::shared_ptr<const Command>> commands;
};

class Environment
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<Environment>;
  using ConstPtr = std::shared_ptr<const Environment>;
  using UPtr = std::unique_ptr<Environment>;
  using ConstUPtr = std::unique_ptr<const Environment>;

  /** @brief Default constructor */
  Environment();
  virtual ~Environment();
  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;
  Environment(Environment&&) = delete;
  Environment& operator=(Environment&&) = delete;

  /**
   * @brief Initialize the Environment
   *
   * The template class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  bool init(const std::vector<std::shared_ptr<const Command>>& commands);

  /**
   * @brief Initialize the Environment
   *
   * The template class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  bool init(const tesseract::scene_graph::SceneGraph& scene_graph,
            const std::shared_ptr<const tesseract::srdf::SRDFModel>& srdf_model = nullptr);

  bool init(const std::string& urdf_string, const std::shared_ptr<const tesseract::common::ResourceLocator>& locator);

  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            const std::shared_ptr<const tesseract::common::ResourceLocator>& locator);

  bool init(const std::filesystem::path& urdf_path,
            const std::shared_ptr<const tesseract::common::ResourceLocator>& locator);

  bool init(const std::filesystem::path& urdf_path,
            const std::filesystem::path& srdf_path,
            const std::shared_ptr<const tesseract::common::ResourceLocator>& locator);

  /**
   * @brief Clone the environment
   * @return A clone of the environment
   */
  Environment::UPtr clone() const;

  /**
   * @brief reset to initialized state
   * @details If the environment has not been initialized then this returns false
   * @return True if environment was successfully reset, otherwise false.
   */
  bool reset();

  /** @brief clear content and uninitialized */
  void clear();

  /** @brief check if the environment is initialized */
  bool isInitialized() const;

  /**
   * @brief Get the current revision number
   * @return Revision number
   */
  int getRevision() const;

  /**
   * @brief Get the initialization revision number
   * @return Initialization revision number
   */
  int getInitRevision() const;

  /**
   * @brief Get the external (client-visible) revision number
   * @details T-398 §7a: external_revision = history_offset + getRevision(). Equal to getRevision()
   * until this Environment's history is first compacted (via a future compactHistory() call);
   * monotonic across the whole object lifetime even though getRevision() itself can decrease when
   * that happens. getRevision() itself is intentionally left meaning the internal (compact) value -
   * existing callers of getRevision() must keep seeing that, unchanged.
   * @return External revision number
   */
  std::int64_t getExternalRevision() const;

  /**
   * @brief Set this Environment's external-revision offset (T-398 §7a §2.7, client bookkeeping)
   * @details For a monitored CLIENT's own Environment object, after a full re-fetch + init(commands)
   * succeeds: call setRevisionOffset(server_external_revision - getRevision()) so this object's own
   * getExternalRevision() then reports the same number the server does. Bookkeeping then advances
   * automatically on every subsequent applyCommands()/processMsg() call, since getRevision() itself
   * already does - no further bookkeeping needed at each call site. Not used by a server-side
   * Environment - its own offset only ever changes via compactHistory()'s internal swap logic.
   * @param offset The offset such that external_revision = offset + getRevision() going forward
   */
  void setRevisionOffset(std::int64_t offset);

  /**
   * @brief Get Environment command history post initialization
   * @return List of commands
   */
  std::vector<std::shared_ptr<const Command>> getCommandHistory() const;

  /**
   * @brief Get the internal command-history length, without copying the history itself
   * @details T-398 §7a: a cheap size check for a compaction trigger predicate - getCommandHistory()
   * copies the whole vector, which is wasteful for a per-tick "is it time to compact?" check.
   * @return commands.size() - always equal to getRevision() for a well-formed Environment
   */
  std::size_t getHistoryLength() const;

  /**
   * @brief Get the command history and external revision as ONE atomic snapshot (T-398 §7a,
   * fable's root-cause fix 2026-09-20)
   * @details getCommandHistory() and getExternalRevision() each take their OWN brief shared_lock -
   * calling them separately (as GetEnvironmentInformation's handler originally did) leaves a gap
   * where a concurrent applyCommands() can land between the two reads. Under live traffic this
   * means the returned history can be k commands FRESHER than the revision number describing it -
   * offset math built from that pair understates by k, and the resulting client mirror's own
   * external revision drifts ahead of the server's, tripping the < branch's reset() (which zeroes
   * the offset) on the very next message and producing a below-floor refusal -> reinit -> repeat
   * loop under continued traffic (measured live: 13, then 8, iterations before this fix - the
   * queued-stale-message fix alone brought 13 down to 8, this closes the remaining gap). Same
   * grouped-read discipline getChangesSince() already uses for its own offset/floor/revision read.
   * @return {command history, external revision} as of the same single lock acquisition
   */
  std::pair<std::vector<std::shared_ptr<const Command>>, std::int64_t> getCommandHistoryAndExternalRevision() const;

  /**
   * @brief Get the commands applied since a client-supplied external revision (T-398 §7a §2.3)
   * @details Single-lock read of history_offset/floor_revision/revision/commands as one consistent
   * group - the read this design's whole point is to make atomic, not a nested sequence of
   * independently-locked getter calls. Refuses (success=false) if external_rev is either ahead of
   * the current external revision (an impossible/stale-client case, unchanged from today) or below
   * floor_revision (the new A3 fix - never silently answers from a truncated history). A refusal's
   * own external_revision field is always the current value, usable as a resync hint either way.
   * @param external_rev The client's own last-known external revision
   * @return success, the current external revision, this Environment's name, and (only on success)
   * the commands applied since external_rev
   */
  EnvironmentChanges getChangesSince(std::int64_t external_rev) const;

  /**
   * @brief Compact this Environment's command history, bounding its unbounded growth (T-398 §7a §2.5/§2.5a)
   * @details Builds a fresh, equivalent Environment by replaying commands[0, keep_from) - the
   * portion of history OLDER than the retained tail - into a temporary Environment, then building
   * the compact baseline (scene graph clone + kinematics info + contact-manager plugin info +
   * allowed-collision matrix + collision margins + joint state) from THAT replayed state, then
   * re-applying the retained tail commands[keep_from, revision) on top as their own original,
   * lightweight command objects (build-then-swap, never swap-then-verify - any failed build step
   * discards the attempt and leaves this Environment completely untouched), then swaps it in under
   * a single unique_lock. A swap-time CAS re-applies any commands that landed in the
   * capture-to-swap gap before swapping (or aborts this attempt entirely if that re-apply itself
   * fails); the swap also re-reads and re-applies the current joint state fresh, since setState()
   * does not bump revision and so is invisible to the CAS above. getExternalRevision() is
   * unaffected by a successful compaction - history_offset absorbs exactly the amount of history
   * discarded, so external_revision only ever reads as continuous, never as a jump.
   * @details T-489 retention (fable's design, 2026-09-27): floor_revision is set to
   * history_offset + baseline_len, NOT the current external revision - a client whose last-known
   * external revision falls anywhere within the retained tail gets served real delta commands via
   * getChangesSince() instead of being refused into a full baseline refetch. This is what bounds
   * both the per-call refetch cost (which otherwise grows with every part ever placed, since the
   * old always-refuse-below-current floor forced a full baseline fetch for any client that was
   * even one poll interval behind) and the resulting permanent per-compaction memory step in every
   * fetcher. keep_from itself is clamped to never fall before the CURRENT baseline's own boundary
   * (floor_revision - history_offset, the previous compaction's own baseline_len) - replaying into
   * the middle of a still-live baseline's own commands would rebuild an incomplete/incorrect
   * intermediate environment.
   * @details Same builder for both the SERVER's own periodic trigger and each MONITORED CLIENT's
   * own trigger against its own Environment object (§2.5a) - there is only one compaction
   * mechanism, exposed as this one public entry point, not two.
   * @param retain_tail how many of the most recent commands to keep uncompacted (as their
   * original, lightweight objects) rather than folding into the baseline. Should be at least
   * ~2x the expected client poll interval expressed in revisions, so a normally-paced client never
   * falls below the new floor. Default 64 keeps the one existing call site (ROSEnvironmentMonitor's
   * periodic trigger) compiling unchanged; make it configurable there (a ROS param), not here.
   * @return true if a compaction actually happened (built and swapped); false if this Environment
   * is not initialized, if the entire history is already within the retention window (nothing old
   * enough to compact), or if any build/swap step failed (logged, this Environment is left exactly
   * as it was, safe to retry on the next call)
   */
  bool compactHistory(std::size_t retain_tail = 64);

  /**
   * @brief Release the retired Implementation a prior compactHistory() call is still holding alive
   * @details T-398 §7a (fable, 2026-09-19 17:0x): the depth-1 retirement queue exists to give an
   * in-flight getName() caller a one-swap safety margin, but the retiree is the FULL prior
   * generation (entire history + every part mesh it carried) - holding it until the NEXT
   * compaction means the compaction that was supposed to free memory frees nothing for a whole
   * trigger-threshold interval. Call this once, one tick after compactHistory() last returned
   * true (one tick is a sufficient safety margin - no known caller holds a getName() reference
   * across a publish period), not at the next compaction. A no-op if nothing is currently retired.
   */
  void releaseRetiredHistory();

  /**
   * @brief Applies the commands to the environment
   * @param commands Commands to be applied to the environment
   * @return true if successful. If returned false, then only a partial set of commands have been applied. Call
   * getCommandHistory to check. Some commands are not checked for success
   */
  bool applyCommands(const std::vector<std::shared_ptr<const Command>>& commands);

  /**
   * @brief Apply command to the environment
   * @param command Command to be applied to the environment
   * @return true if successful. If returned false, then the command have not been applied.
   * Some type of Command are not checked for success
   */
  bool applyCommand(std::shared_ptr<const Command> command);

  /**
   * @brief Get the Scene Graph
   * @return SceneGraphConstPtr
   */
  std::shared_ptr<const tesseract::scene_graph::SceneGraph> getSceneGraph() const;

  /**
   * @brief Get a groups joint names
   * @param group_name The group name
   * @return A vector of joint names
   */
  std::vector<std::string> getGroupJointNames(const std::string& group_name) const;

  /**
   * @brief Get a joint group by name
   * @param group_name The group name
   * @return A joint group
   */
  std::shared_ptr<const tesseract::kinematics::JointGroup> getJointGroup(const std::string& group_name) const;

  /**
   * @brief Get a joint group given a vector of joint names
   * @param name The name to assign to the joint group
   * @param joint_names The joint names that make up the group
   * @return A joint group
   */
  std::shared_ptr<const tesseract::kinematics::JointGroup>
  getJointGroup(const std::string& name, const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get a kinematic group given group name and solver name
   * @details If ik_solver_name is empty it will choose the first ik solver for the group
   * @param group_name The group name
   * @param ik_solver_name The IK solver name
   * @return A kinematics group
   */
  std::shared_ptr<const tesseract::kinematics::KinematicGroup>
  getKinematicGroup(const std::string& group_name, const std::string& ik_solver_name = "") const;

  /**
   * @brief Find tool center point provided in the manipulator info
   *
   * If manipulator information tcp is defined as a string it does the following
   *    - First check if manipulator info is empty or already an Isometry3d, if so return identity
   *    - Next if not, it checks if the tcp offset name is a link in the environment if so throw an exception.
   *    - Next if not found, it looks up the tcp name in the SRDF kinematics information
   *    - Next if not found, it leverages the user defined callbacks to try an locate the tcp information.
   *    - Next throw an exception, because no tcp information was located.
   *
   * @param manip_info The manipulator info
   * @return The tool center point
   */
  Eigen::Isometry3d findTCPOffset(const tesseract::common::ManipulatorInfo& manip_info) const;

  /**
   * @brief This allows for user defined callbacks for looking up TCP information
   * @param fn User defined callback function for locating TCP information
   */
  void addFindTCPOffsetCallback(const FindTCPOffsetCallbackFn& fn);

  /**
   * @brief This get the current find tcp callbacks stored in the environment
   * @return A vector of callback functions
   */
  std::vector<FindTCPOffsetCallbackFn> getFindTCPOffsetCallbacks() const;

  /**
   * @brief Add an event callback function
   * @details When these get called they are protected by a unique lock internally so if the
   * callback is a long event it can impact performance.
   * @note These do not get cloned or serialized
   * @param hash The id associated with the callback to allow removal. It is recommended to use
   * std::hash<Object*>{}(this) to associate the callback with the class it associated with.
   * @param fn User defined callback function which gets called for different event triggers
   */
  void addEventCallback(std::size_t hash, const EventCallbackFn& fn);

  /**
   * @brief Remove event callbacks
   * @param hash the id associated with the callback to be removed
   */
  void removeEventCallback(std::size_t hash);

  /** @brief clear all event callbacks */
  void clearEventCallbacks();

  /**
   * @brief Get the current event callbacks stored in the environment
   * @return A map of callback functions
   */
  std::map<std::size_t, EventCallbackFn> getEventCallbacks() const;

  /**
   * @brief Set resource locator for environment
   * @param locator The resource locator
   */
  void setResourceLocator(std::shared_ptr<const tesseract::common::ResourceLocator> locator);

  /**
   * @brief Get the resource locator assigned
   * @details This can be a nullptr
   * @return The resource locator assigned to the environment
   */
  std::shared_ptr<const tesseract::common::ResourceLocator> getResourceLocator() const;

  /** @brief Give the environment a name */
  void setName(const std::string& name);

  /** @brief Get the name of the environment
   *
   * This may be empty, if so check urdf name
   */
  const std::string& getName() const;

  /**
   * @brief Set the current state of the environment
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  void setState(const std::unordered_map<std::string, double>& joints,
                const tesseract::common::TransformMap& floating_joints = {});
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                const tesseract::common::TransformMap& floating_joints = {});

  /**
   * @brief Set the current state of the floating joint values
   * @param floating_joint_values The floating joint values to set
   */
  void setState(const tesseract::common::TransformMap& floating_joints);

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  tesseract::scene_graph::SceneState getState(const std::unordered_map<std::string, double>& joints,
                                              const tesseract::common::TransformMap& floating_joints = {}) const;
  tesseract::scene_graph::SceneState getState(const std::vector<std::string>& joint_names,
                                              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                              const tesseract::common::TransformMap& floating_joints = {}) const;

  /**
   * @brief Get the state given floating joint values
   * @param floating_joint_values The floating joint values to leverage
   * @return A the state of the environment
   */
  tesseract::scene_graph::SceneState getState(const tesseract::common::TransformMap& floating_joints) const;

  /** @brief Get the current state of the environment */
  tesseract::scene_graph::SceneState getState() const;

  /**
   * @brief Get the link transforms of the scene for a given set or subset of joint values.
   *
   * This is provided to optimize motion planning where link_transforms are poplated multiple time
   *
   * This does not change the internal state of the solver.
   *
   * @param link_transforms The link_transforms to populate with data.
   * @param joints A map of joint names to joint values to change.
   * @param joint_values The joint values
   * @param floating_joints The floating joint origin transform
   */
  void getLinkTransforms(tesseract::common::TransformMap& link_transforms,
                         const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                         const tesseract::common::TransformMap& floating_joints) const;

  /**
   * @brief Get the link transforms of the scene for a given set or subset of joint values.
   *
   * This is provided to optimize motion planning where link_transforms are poplated multiple time
   *
   * This does not change the internal state of the solver.
   *
   * @param link_transforms The link_transforms to populate with data.
   * @param joints A map of joint names to joint values to change.
   * @param joint_values The joint values
   */
  void getLinkTransforms(tesseract::common::TransformMap& link_transforms,
                         const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** @brief Last update time. Updated when any change to the environment occurs */
  std::chrono::system_clock::time_point getTimestamp() const;

  /** @brief Last update time to current state. Updated only when current state is updated */
  std::chrono::system_clock::time_point getCurrentStateTimestamp() const;

  /**
   * @brief Get a link in the environment
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  std::shared_ptr<const tesseract::scene_graph::Link> getLink(const std::string& name) const;

  /**
   * @brief Get joint by name
   * @param name The name of the joint
   * @return Joint Const Pointer
   */
  std::shared_ptr<const tesseract::scene_graph::Joint> getJoint(const std::string& name) const;

  /**
   * @brief Gets the limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @return The joint limits set for the given joint
   */
  std::shared_ptr<const tesseract::scene_graph::JointLimits> getJointLimits(const std::string& joint_name) const;

  /**
   * @brief Get whether a link should be considered during collision checking
   * @return True if should be considered during collision checking, otherwise false
   */
  bool getLinkCollisionEnabled(const std::string& name) const;

  /**
   * @brief Get a given links visibility setting
   * @return True if should be visible, otherwise false
   */
  bool getLinkVisibility(const std::string& name) const;

  /**
   * @brief Get the allowed collision matrix
   * @return AllowedCollisionMatrixConstPtr
   */
  std::shared_ptr<const tesseract::common::AllowedCollisionMatrix> getAllowedCollisionMatrix() const;

  /**
   * @brief Get a vector of joint names in the environment
   * @return A vector of joint names
   */
  std::vector<std::string> getJointNames() const;

  /**
   * @brief Get a vector of active joint names in the environment
   * @return A vector of active joint names
   */
  std::vector<std::string> getActiveJointNames() const;

  /**
   * @brief Get the current state of the environment
   *
   * Order should be the same as getActiveJointNames()
   *
   * @return A vector of joint values
   */
  Eigen::VectorXd getCurrentJointValues() const;

  /**
   * @brief Get the current joint values for a vector of joints
   *
   * Order should be the same as the input vector
   *
   * @return A vector of joint values
   */
  Eigen::VectorXd getCurrentJointValues(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get the current floating joint values
   * @return The joint origin transform for the floating joint
   */
  tesseract::common::TransformMap getCurrentFloatingJointValues() const;

  /**
   * @brief Get the current floating joint values
   * @return The joint origin transform for the floating joint
   */
  tesseract::common::TransformMap getCurrentFloatingJointValues(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get the root link name
   * @return String
   */
  std::string getRootLinkName() const;

  /**
   * @brief Get a vector of link names in the environment
   * @return A vector of link names
   */
  std::vector<std::string> getLinkNames() const;

  /**
   * @brief Get a vector of active link names in the environment
   * @return A vector of active link names
   */
  std::vector<std::string> getActiveLinkNames() const;

  /**
   * @brief Get a vector of active link names affected by the provided joints in the environment
   * @param joint_names A list of joint names
   * @return A vector of active link names
   */
  std::vector<std::string> getActiveLinkNames(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get a vector of static link names in the environment
   * @return A vector of static link names
   */
  std::vector<std::string> getStaticLinkNames() const;

  /**
   * @brief Get a vector of static link names not affected by the provided joints in the environment
   * @param joint_names A list of joint names
   * @return A vector of static link names
   */
  std::vector<std::string> getStaticLinkNames(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get all of the links transforms
   *
   * Order should be the same as getLinkNames()
   *
   * @return Get a vector of transforms for all links in the environment.
   */
  tesseract::common::VectorIsometry3d getLinkTransforms() const;

  /**
   * @brief Get the transform corresponding to the link.
   * @return Transform and is identity when no transform is available.
   */
  Eigen::Isometry3d getLinkTransform(const std::string& link_name) const;

  /**
   * @brief Get transform between two links using the current state
   * @param from_link_name The link name the transform should be relative to
   * @param to_link_name The link name to get transform
   * @return The relative transform = inv(Transform(from_link_name)) * Transform(to_link_name)
   */
  Eigen::Isometry3d getRelativeLinkTransform(const std::string& from_link_name, const std::string& to_link_name) const;

  /**
   * @brief Returns a clone of the environments state solver
   *
   * The Environment::getState contains mutex's which is may not be needed in all motion planners. This allows the user
   * to get snap shot of the environment to calculate the state.
   *
   * @return A clone of the environments state solver
   */
  std::unique_ptr<tesseract::scene_graph::StateSolver> getStateSolver() const;

  /**
   * @brief Get the kinematics information
   * @return The kinematics information
   */
  tesseract::srdf::KinematicsInformation getKinematicsInformation() const;

  /**
   * @brief Get the available group names
   * @return The group names
   */
  std::set<std::string> getGroupNames() const;

  /**
   * @brief Get the contact managers plugin information
   * @return The contact managers plugin information
   */
  tesseract::common::ContactManagersPluginInfo getContactManagersPluginInfo() const;

  /**
   * @brief Set the active discrete contact manager
   * @param name The name used to register the contact manager
   * @return True of name exists in DiscreteContactManagerFactory
   */
  bool setActiveDiscreteContactManager(const std::string& name);

  /** @brief Get a copy of the environments active discrete contact manager */
  std::unique_ptr<tesseract::collision::DiscreteContactManager> getDiscreteContactManager() const;

  /**
   * @brief Set the cached internal copy of the environments active discrete contact manager not nullptr
   * @details This can be useful to save space in the event the environment is being saved
   */
  void clearCachedDiscreteContactManager() const;

  /** @brief Get a copy of the environments available discrete contact manager by name */
  std::unique_ptr<tesseract::collision::DiscreteContactManager>
  getDiscreteContactManager(const std::string& name) const;

  /**
   * @brief Set the active continuous contact manager
   * @param name The name used to register the contact manager
   * @return True of name exists in ContinuousContactManagerFactory
   */
  bool setActiveContinuousContactManager(const std::string& name);

  /** @brief Get a copy of the environments active continuous contact manager */
  std::unique_ptr<tesseract::collision::ContinuousContactManager> getContinuousContactManager() const;

  /**
   * @brief Set the cached internal copy of the environments active continuous contact manager not nullptr
   * @details This can be useful to save space in the event the environment is being saved
   */
  void clearCachedContinuousContactManager() const;

  /** @brief Get a copy of the environments available continuous contact manager by name */
  std::unique_ptr<tesseract::collision::ContinuousContactManager>
  getContinuousContactManager(const std::string& name) const;

  /** @brief Get the environment collision margin data */
  tesseract::common::CollisionMarginData getCollisionMarginData() const;

  /**
   * @brief Lock the environment when wanting to make multiple reads
   * @details This is useful when making multiple read function calls and don't want the data
   * to get updated between function calls when there a multiple threads updating a single environment
   */
  std::shared_lock<std::shared_mutex> lockRead() const;

  /**
   * @brief These operators are to facilitate checking serialization but may have value elsewhere
   * @param rhs The environment to compare
   * @return True if they are equal otherwise false
   */
  bool operator==(const Environment& rhs) const;
  bool operator!=(const Environment& rhs) const;

private:
  /** @brief The environment can be accessed from multiple threads, need use mutex throughout */
  mutable std::shared_mutex mutex_;

  /** @brief This hides specific implemenation details to allow forward declarations */
  struct Implementation;
  std::unique_ptr<Implementation> impl_;

  /** @brief This is provided for serialization */
  void init(const std::vector<std::shared_ptr<const Command>>& commands,
            int init_revision,
            const std::chrono::system_clock::time_point& timestamp,
            const tesseract::scene_graph::SceneState& current_state,
            const std::chrono::system_clock::time_point& current_state_timestamp,
            const std::shared_ptr<const tesseract::common::ResourceLocator>& resource_locator);

  template <class Archive>
  friend void ::tesseract::environment::serialize(Archive& ar, Environment& obj);

public:
  /** @brief This should only be used by the clone method */
  explicit Environment(std::unique_ptr<Implementation> impl);
};

using EnvironmentPtrAnyPoly = tesseract::common::AnyWrapper<std::shared_ptr<tesseract::environment::Environment>>;
using EnvironmentConstPtrAnyPoly =
    tesseract::common::AnyWrapper<std::shared_ptr<const tesseract::environment::Environment>>;
}  // namespace tesseract::environment

#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_H
