#ifndef SIMPLE_NODE_H
#define SIMPLE_NODE_H

#include "gloo/SceneNode.hpp"
#include "ParticleState.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorType.hpp"
#include "gloo/utils.hpp"
#include "gloo/InputManager.hpp"
#include "gloo/MeshLoader.hpp"

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"

#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/InputManager.hpp"

#include "ForwardEulerIntegrator.hpp"

#include "IntegratorFactory.hpp"
#include "IntegratorType.hpp"
#include "SimpleSystem.hpp"

#include <fstream>

#include <string>
#include <stdexcept>
#include <vector>

namespace GLOO {
template <class TSystem, class TState> // <SimpleSystem, ParticleState>
class SimpleNode : public SceneNode {
public:
    SimpleNode(float integration_step, IntegratorType integrator_type){
        glm::vec3 initial_position(-10,1,1);

        std::shared_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.5f, 20, 20);
        std::shared_ptr<ShaderProgram> shader = std::make_shared<PhongShader>();
        auto sphere_color = glm::vec3(0.f, 0.f, 1.f); // blue
        auto sphere_material = std::make_shared<Material>(sphere_color, sphere_color, sphere_color, 0);


        auto sphere_node = make_unique<SceneNode>(); 
        sphere_node->GetTransform().SetPosition(initial_position);
        sphere_node->CreateComponent<ShadingComponent>(shader);
        sphere_node->CreateComponent<RenderingComponent>(sphere_mesh);
        sphere_node->CreateComponent<MaterialComponent>(sphere_material);

        sphere_node_ = sphere_node.get();
        AddChild(std::move(sphere_node));


        std::vector<glm::vec3> positions_vec;
        positions_vec.push_back(initial_position);
        std::vector<glm::vec3> velocities_vec; //will be ignored in this simplenode
        velocities_vec.push_back(glm::vec3(0,0,0));
        state_.positions = positions_vec;
        state_.velocities = velocities_vec;

        system_ = SimpleSystem();

        integration_step_ = integration_step;
        integrator_type_ = integrator_type;
        integrator_ = IntegratorFactory::CreateIntegrator<TSystem, TState>(integrator_type_);
        time_ = 0;
    };

    void Update(double delta_time) override{ 
      auto new_state = integrator_->Integrate(system_, state_, time_, integration_step_);
      sphere_node_->GetTransform().SetPosition(new_state.positions[0]);
      state_ = new_state;

      time_ = time_ + delta_time;
    };


private:
    float integration_step_;
    SceneNode* sphere_node_;
    ParticleState state_;
    SimpleSystem system_;

    double time_;
    std::unique_ptr<IntegratorBase<TSystem, TState>> integrator_;
    IntegratorType integrator_type_;
};

} // namespace GLOO

#endif