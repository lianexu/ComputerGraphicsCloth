#ifndef PENDULUM_NODE_H
#define PENDULUM_NODE_H

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
#include "PendulumSystem.hpp"

#include <fstream>

#include <string>
#include <stdexcept>
#include <vector>

#include "glm/ext.hpp" //for printing

namespace GLOO {
template <class TSystem, class TState> // <SimpleSystem, ParticleState>
class PendulumNode : public SceneNode {
public:
    PendulumNode(float integration_step, IntegratorType integrator_type){
        system_ = PendulumSystem();

        std::vector<glm::vec3> positions_vec;
        std::vector<glm::vec3> velocities_vec;

        std::shared_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.05f, 20, 20);
        std::shared_ptr<ShaderProgram> shader = std::make_shared<PhongShader>();

        auto sphere_node0 = make_unique<SceneNode>(); 
        sphere_node0->GetTransform().SetPosition(glm::vec3(0,0,0));
        system_.AddSphere(-1, -1, 0, 0, 0, 0, 0); //int sphere1, int sphere2, float mass, float spring_length1k, float spring_constant1k, float spring_lengthk2, float spring_constantk2
        positions_vec.push_back(glm::vec3(0,0,0)); //POSITION
        velocities_vec.push_back(glm::vec3(0,0,0)); //VELOCITY
        sphere_node0->CreateComponent<ShadingComponent>(shader);
        sphere_node0->CreateComponent<RenderingComponent>(sphere_mesh);
        sphere_nodes_.push_back(sphere_node0.get());
        AddChild(std::move(sphere_node0));

        auto sphere_node1 = make_unique<SceneNode>(); 
        sphere_node1->GetTransform().SetPosition(glm::vec3(0,-1,0));
        positions_vec.push_back(glm::vec3(0,-1,0)); // POSITION
        velocities_vec.push_back(glm::vec3(0,-0.2,0)); //VELOCITY
        system_.AddSphere(0, -1, 0.005, 0.5, 0.2, 0, 0);
        sphere_node1->CreateComponent<ShadingComponent>(shader);
        sphere_node1->CreateComponent<RenderingComponent>(sphere_mesh);
        sphere_nodes_.push_back(sphere_node1.get());
        AddChild(std::move(sphere_node1));

        auto sphere_node2 = make_unique<SceneNode>(); 
        sphere_node2->GetTransform().SetPosition(glm::vec3(0,-1,0));
        positions_vec.push_back(glm::vec3(2,-2,0)); // POSITION
        velocities_vec.push_back(glm::vec3(0,0.2,0)); //VELOCITY
        system_.AddSphere(1, -1, 0.005, 0.5, 0.2, 0, 0);
        sphere_node2->CreateComponent<ShadingComponent>(shader);
        sphere_node2->CreateComponent<RenderingComponent>(sphere_mesh);
        sphere_nodes_.push_back(sphere_node2.get());
        AddChild(std::move(sphere_node2));

        auto sphere_node3 = make_unique<SceneNode>(); 
        sphere_node3->GetTransform().SetPosition(glm::vec3(0,-1,0));
        positions_vec.push_back(glm::vec3(3,2,0)); // POSITION
        velocities_vec.push_back(glm::vec3(0,1,0)); //VELOCITY
        system_.AddSphere(2, -1, 0.005, 0.5, 0.2, 0, 0);
        sphere_node3->CreateComponent<ShadingComponent>(shader);
        sphere_node3->CreateComponent<RenderingComponent>(sphere_mesh);
        sphere_nodes_.push_back(sphere_node3.get());
        AddChild(std::move(sphere_node3));

        system_.FixSphere(0);

        state_.positions = positions_vec;
        state_.velocities = velocities_vec;


        integration_step_ = integration_step;
        integrator_type_ = integrator_type;
        integrator_ = IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type_);
        time_ = 0;
    };

    void Update(double delta_time) override{ 
      auto new_state = integrator_->Integrate(system_, state_, time_, integration_step_);
      sphere_nodes_[0]->GetTransform().SetPosition(new_state.positions[0]);
      sphere_nodes_[1]->GetTransform().SetPosition(new_state.positions[1]);
      sphere_nodes_[2]->GetTransform().SetPosition(new_state.positions[2]);
      sphere_nodes_[3]->GetTransform().SetPosition(new_state.positions[3]);

      state_ = new_state;

      time_ = time_ + delta_time;
    };


private:
    float integration_step_;
    std::vector<SceneNode*> sphere_nodes_;
    ParticleState state_;
    PendulumSystem system_;
    // PendulumSystem system_;


    std::vector<std::vector<float>> spring_constants_;
    std::vector<std::vector<float>> spring_lengths_;
     std::vector<float> sphere_masses_;
    //  float drag_constant_ = 0.005;

    double time_;
    std::unique_ptr<IntegratorBase<TSystem, TState>> integrator_;
    IntegratorType integrator_type_;
};

} // namespace GLOO

#endif