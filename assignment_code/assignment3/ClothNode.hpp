#ifndef CLOTH_NODE_H
#define CLOTH_NODE_H

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

// #include "ForwardEulerIntegrator.hpp"

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
class ClothNode : public SceneNode {
public:
    ClothNode(float integration_step, IntegratorType integrator_type){
        auto cloth_positions = make_unique<PositionArray>(); //starter code
        auto cloth_normals = make_unique<NormalArray>(); //starter code
        auto cloth_indices = make_unique<IndexArray>(); //starter code

        system_ = PendulumSystem();

        std::vector<glm::vec3> positions_vec;
        std::vector<glm::vec3> velocities_vec;

        std::shared_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.05f, 20, 20);
        std::shared_ptr<ShaderProgram> shader = std::make_shared<PhongShader>();

        for (int i = 0; i < cloth_dimensions_*cloth_dimensions_; i++){ // add 9 nodes
            auto sphere_node = make_unique<SceneNode>(); 
            sphere_node->CreateComponent<ShadingComponent>(shader);

            // auto& rc = patch_node->CreateComponent<RenderingComponent>(patch_mesh_);
            // rc.SetDrawMode(DrawMode::Triangles);

            auto& rc = sphere_node->CreateComponent<RenderingComponent>(sphere_mesh);
            rc.SetDrawMode(DrawMode::Lines);

            sphere_nodes_.push_back(sphere_node.get());
            AddChild(std::move(sphere_node));
        }
        for (int i = 0; i < cloth_dimensions_; i++){
            for (int j = 0; j < cloth_dimensions_; j++){
                glm::vec3 node_position(j,-i,i*2);
                int sphere_ix = IndexOf(i,j);
                sphere_nodes_[sphere_ix]->GetTransform().SetPosition(node_position);
                positions_vec.push_back(node_position); //POSITION
                velocities_vec.push_back(glm::vec3(0,0,0)); //VELOCITY    
                system_.AddSphere(0.005);
                if (i == 0){
                    system_.FixSphere(sphere_ix);
                }

                cloth_normals->push_back(glm::vec3(0,0,0));
                cloth_positions->push_back(node_position);
            }
        }


        for (int i = 0; i < cloth_dimensions_-1; i++){
            for (int j = 0; j < cloth_dimensions_-1; j++){
                int this_sphere = IndexOf(i,j);
                int sphere_right = IndexOf(i, j+1);
                int sphere_down = IndexOf(i+1, j);
                int sphere_down_right = IndexOf(i+1, j+1);
                int sphere_down_left = IndexOf(i+1, j-1);

                cloth_indices->push_back(this_sphere);
                cloth_indices->push_back(sphere_down_right);
                cloth_indices->push_back(sphere_down);

                cloth_indices->push_back(this_sphere);
                cloth_indices->push_back(sphere_down_right);
                cloth_indices->push_back(sphere_right);
            }
        }
        // std::shared_ptr<VertexObject> patch_mesh_ = std::make_shared<VertexObject>();
        patch_mesh_->UpdatePositions(std::move(cloth_positions)); //starter code
        patch_mesh_->UpdateNormals(std::move(cloth_normals)); //starter code
        patch_mesh_->UpdateIndices(std::move(cloth_indices)); //starter code


        // auto shader = std::make_shared<SimpleShader>(); 
        std::unique_ptr<GLOO::SceneNode> patch_node = make_unique<SceneNode>();
        patch_node->CreateComponent<ShadingComponent>(shader);
        auto& rc = patch_node->CreateComponent<RenderingComponent>(patch_mesh_);
        rc.SetDrawMode(DrawMode::Triangles);

        AddChild(std::move(patch_node));


        // Adding the springs
        for (int i = 0; i < cloth_dimensions_; i++){
            for (int j = 0; j < cloth_dimensions_; j++){
                int this_sphere = IndexOf(i,j);
                int sphere_right = IndexOf(i, j+1);
                int sphere_down = IndexOf(i+1, j);
                int sphere_down_right = IndexOf(i+1, j+1);
                int sphere_down_left = IndexOf(i+1, j-1);

                // Structural Springs
                if (i == cloth_dimensions_-1 && j == cloth_dimensions_ -1){
                    // do nothing
                }else if (i == cloth_dimensions_-1){
                    system_.AddSpring(this_sphere, sphere_right, 1, 0.2);
                }else if (j == cloth_dimensions_-1){
                    system_.AddSpring(this_sphere, sphere_down, 1, 0.2);
                }else{
                    system_.AddSpring(this_sphere, sphere_right, 1, 0.2);
                    system_.AddSpring(this_sphere, sphere_down, 1, 0.2);
                }

                // Shear Springs
                if (i == cloth_dimensions_-1){
                    // do nothing
                }else if (j == 0){
                    system_.AddSpring(this_sphere, sphere_down_right, 1, 0.2);
                }else if (j == cloth_dimensions_-1){
                    system_.AddSpring(this_sphere, sphere_down_left, 1, 0.2);
                }else{
                    system_.AddSpring(this_sphere, sphere_down_right, 1, 0.2);
                    system_.AddSpring(this_sphere, sphere_down_left, 1, 0.2);
                }

                // Flex Springs
                int sphere_2right = IndexOf(i, j+2);
                int sphere_2down = IndexOf(i+2, j);

                if (i >= cloth_dimensions_-2 && j >= cloth_dimensions_ -2){
                    // do nothing
                }else if (j >= cloth_dimensions_ -2){
                    system_.AddSpring(this_sphere, sphere_2down, 2, 0.2);
                }else if (i >= cloth_dimensions_-2){
                    system_.AddSpring(this_sphere, sphere_2right, 2, 0.2);
                }else{
                    system_.AddSpring(this_sphere, sphere_2right, 2, 0.2);
                    system_.AddSpring(this_sphere, sphere_2down, 2, 0.2);
                }
            }
        }

        state_.positions = positions_vec;
        state_.velocities = velocities_vec;

        integration_step_ = integration_step;
        integrator_type_ = integrator_type;
        integrator_ = IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type_);
        time_ = 0;
    };

    int IndexOf(int i, int j){
        return cloth_dimensions_ * i + j;
    }

    void Update(double delta_time) override{ 
        auto cloth_positions = make_unique<PositionArray>();
      auto new_state = integrator_->Integrate(system_, state_, time_, integration_step_);
      for (int i = 0; i < sphere_nodes_.size(); i++){
        sphere_nodes_[i]->GetTransform().SetPosition(new_state.positions[i]);
        cloth_positions->push_back(new_state.positions[i]);
      }
      state_ = new_state;
      patch_mesh_->UpdatePositions(std::move(cloth_positions));

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

    int cloth_dimensions_ = 4;
    std::shared_ptr<VertexObject> patch_mesh_ = std::make_shared<VertexObject>();
 
};

} // namespace GLOO

#endif