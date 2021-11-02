#pragma once

class TaskUnderstanding;
class SceneUnderstanding;
class SceneObject;

auto detect_objects(TaskUnderstanding& task) -> void;
/// Detects which objects are involved in the task and fills in focus_objects in Task.
auto detect_change(TaskUnderstanding& task) -> void;
auto detect_features(SceneObject& object) -> void;
auto describe_relations(const SceneUnderstanding& scene) -> void;
auto form_hypotheses(const TaskUnderstanding& task) -> void;

auto copy_features(const SceneObject& src, SceneObject& dst) -> void;
