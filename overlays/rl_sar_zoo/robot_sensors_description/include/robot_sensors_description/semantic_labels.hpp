// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/RTShaderSystem.hh>
#include <gazebo/rendering/ogre_gazebo.h>
#include <OgreBitwise.h>
#include <yaml-cpp/yaml.h>
#include <fnmatch.h>

#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_sensors_description
{
// A camera-local material scheme: GUI materials and the depth pass are untouched.
// Gazebo attaches the scoped visual name to each Ogre entity's user data.
class SemanticLabels : public Ogre::MaterialManager::Listener, public Ogre::RenderTargetListener
{
public:
  SemanticLabels(const std::string & path, const std::string & scheme,
    const std::string & world) : scheme_(scheme), world_prefix_(world + "::")
  {
    const auto config = YAML::LoadFile(path);
    if (!config["classes"].IsMap() || !config["rules"].IsSequence()) {
      throw std::runtime_error("Semantic config requires classes and rules");
    }
    for (const auto & entry : config["classes"]) {
      const int id = entry.first.as<int>();
      if (id < 0 || id > 65534 || entry.second.as<std::string>().empty()) {
        throw std::runtime_error("Semantic class IDs must be 0..65534 with names");
      }
      classes_.emplace(id, entry.second.as<std::string>());
    }
    if (classes_.count(0) == 0) {
      throw std::runtime_error("Semantic class 0 must describe other surfaces");
    }
    if (classes_.size() > 2049) {
      throw std::runtime_error("Semantic config supports at most 2049 simultaneous classes");
    }
    for (const auto & entry : config["rules"]) {
      const auto pattern = entry["visual"].as<std::string>();
      const int id = entry["class_id"].as<int>();
      if (pattern.empty() || classes_.count(id) == 0) {
        throw std::runtime_error("Semantic rule references an undefined class");
      }
      rules_.push_back({pattern, static_cast<uint16_t>(id)});
    }
    fragment_ = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
      scheme_ + "/fragment", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
    fragment_->setSource(
      "#version 120\nuniform vec4 labelColour;\n"
      "void main() { gl_FragColor = labelColour; }\n");
    fragment_->load();
    // Half floats store table indices 0..2048 exactly. The lookup table preserves
    // arbitrary external IDs while halving label readback; depth stays float32.
    for (const auto & entry : classes_) {
      const auto id = entry.first;
      auto material = Ogre::MaterialManager::getSingleton().create(
        scheme_ + "/" + std::to_string(id),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      auto * technique = material->getTechnique(0);
      technique->setSchemeName(scheme_);
      auto * pass = technique->getPass(0);
      pass->setLightingEnabled(false);
      pass->setFragmentProgram(fragment_->getName());
      pass->getFragmentProgramParameters()->setNamedConstant("labelColour",
        Ogre::Vector4(static_cast<float>(decode_ids_.size()), 0, 0, 1));
      decode_ids_.push_back(id);
      pass->setFog(true, Ogre::FOG_NONE);
      pass->setSceneBlending(Ogre::SBT_REPLACE);
      pass->setDepthCheckEnabled(true);
      pass->setDepthWriteEnabled(true);
      material->setReceiveShadows(false);
      material->load();
      materials_.emplace(id, material);
    }
  }

  ~SemanticLabels() override
  {
    auto * manager = Ogre::MaterialManager::getSingletonPtr();
    if (manager) {
      manager->removeListener(this, scheme_);
      for (auto * target : targets_) {target->removeListener(this);}
      for (const auto & entry : materials_) {manager->remove(entry.second->getName());}
      Ogre::HighLevelGpuProgramManager::getSingleton().remove(fragment_->getName());
    }
  }

  void Attach(const gazebo::rendering::DepthCameraPtr & camera)
  {
    if (camera->RenderTexture()->getFormat() != Ogre::PF_FLOAT16_R) {
      throw std::runtime_error("Semantic image requires R_FLOAT16 camera format");
    }
    // Reuse the ordinary image pass already rendered by DepthCamera. The depth
    // texture remains FLOAT32_R, with the original measurement/projection math.
    if (camera->RenderTexture()->getFSAA() != 0) {
      // Gazebo 11.10 ignores SDF anti_aliasing on this path. Recreate only the
      // ordinary image target before its first render, preserving the camera
      // and the independent depth texture. Blended IDs are never acceptable.
      auto * texture = camera->RenderTexture();
      gazebo::rendering::RTShaderSystem::DetachViewport(
        camera->OgreViewport(), camera->GetScene());
      texture->getBuffer()->getRenderTarget()->removeAllViewports();
      texture->freeInternalResources();
      texture->setFSAA(0, "");
      texture->createInternalResources();
      camera->SetRenderTarget(texture->getBuffer()->getRenderTarget());
    }
    auto * viewport = camera->OgreViewport();
    viewport->setMaterialScheme(scheme_);
    viewport->setBackgroundColour(Ogre::ColourValue::Black);
    viewport->setShadowsEnabled(false);
    viewport->setSkiesEnabled(false);
    viewport->setOverlaysEnabled(false);
    viewport->setVisibilityMask(
      GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
    auto * target = camera->RenderTexture()->getBuffer()->getRenderTarget();
    if (target->getFSAA() != 0) {
      throw std::runtime_error("Semantic render target still has anti-aliasing enabled");
    }
    target->addListener(this);
    targets_.push_back(target);
  }

  void preRenderTargetUpdate(const Ogre::RenderTargetEvent &) override
  {
    Ogre::MaterialManager::getSingleton().addListener(this, scheme_);
  }

  void postRenderTargetUpdate(const Ogre::RenderTargetEvent &) override
  {
    Ogre::MaterialManager::getSingleton().removeListener(this, scheme_);
  }

  Ogre::Technique * handleSchemeNotFound(unsigned short,
    const Ogre::String & scheme, Ogre::Material *, unsigned short,
    const Ogre::Renderable * renderable) override
  {
    // Gazebo also queries getBestTechnique() for its depth compositor without
    // a renderable. Never replace that global depth shader with a class material.
    if (scheme != scheme_ || !renderable) {return nullptr;}
    uint16_t id = 0;
    const auto * sub = dynamic_cast<const Ogre::SubEntity *>(renderable);
    if (sub) {
      const auto & data = sub->getParent()->getUserObjectBindings().getUserAny();
      if (!data.isEmpty() && data.getType() == typeid(std::string)) {
        id = ClassForVisual(Ogre::any_cast<std::string>(data));
      }
    }
    // Unlabelled geometry still writes depth, so it occludes labelled geometry.
    return materials_.at(id)->getTechnique(0);
  }

  uint16_t ClassForVisual(std::string name)
  {
    if (name.compare(0, world_prefix_.size(), world_prefix_) == 0) {
      name.erase(0, world_prefix_.size());
    }
    const auto cached = cache_.find(name);
    if (cached != cache_.end()) {return cached->second;}
    uint16_t id = 0;
    for (const auto & rule : rules_) {
      if (fnmatch(rule.pattern.c_str(), name.c_str(), 0) == 0) {
        id = rule.id;
        break;
      }
    }
    cache_.emplace(name, id);
    return id;
  }

  uint16_t Decode(const unsigned char * image, std::size_t pixel) const
  {
    uint16_t encoded;
    std::memcpy(&encoded, image + pixel * sizeof(encoded), sizeof(encoded));
    const auto index = static_cast<std::size_t>(Ogre::Bitwise::halfToFloat(encoded));
    return decode_ids_.at(index);
  }

private:
  struct Rule {std::string pattern; uint16_t id;};
  std::string scheme_, world_prefix_;
  std::map<int, std::string> classes_;
  std::vector<Rule> rules_;
  std::vector<uint16_t> decode_ids_;
  std::unordered_map<std::string, uint16_t> cache_;
  std::map<uint16_t, Ogre::MaterialPtr> materials_;
  Ogre::HighLevelGpuProgramPtr fragment_;
  std::vector<Ogre::RenderTarget *> targets_;
};
}  // namespace robot_sensors_description
