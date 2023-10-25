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
        // initialize cloth 
        auto cloth_positions = make_unique<PositionArray>(); 
        auto cloth_normals = make_unique<NormalArray>(); 
        auto cloth_indices = make_unique<IndexArray>();

        system_ = PendulumSystem(); // initialize system

        std::vector<glm::vec3> positions_vec;
        std::vector<glm::vec3> velocities_vec;

        std::shared_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.05f, 20, 20);
        std::shared_ptr<ShaderProgram> shader = std::make_shared<PhongShader>();

        // initialize spheres
        for (int i = 0; i < cloth_dimensions_*cloth_dimensions_; i++){ 
            auto sphere_node = make_unique<SceneNode>(); 
            sphere_node->CreateComponent<ShadingComponent>(shader);
            auto& rc = sphere_node->CreateComponent<RenderingComponent>(sphere_mesh);
            rc.SetDrawMode(DrawMode::Lines);

            sphere_nodes_.push_back(sphere_node.get());
            AddChild(std::move(sphere_node));
        }

        
        for (int i = 0; i < cloth_dimensions_; i++){ // add spheres to system
            for (int j = 0; j < cloth_dimensions_; j++){
                glm::vec3 node_position(j*0.5,-i*0.05,i*1);
                int sphere_ix = IndexOf(i,j);
                sphere_nodes_[sphere_ix]->GetTransform().SetPosition(node_position);
                positions_vec.push_back(node_position); //POSITION

                glm::vec3 node_velocity(0,0,0);
                velocities_vec.push_back(node_velocity); //VELOCITY    
                system_.AddSphere(0.005);
                if (i == 0){
                    system_.FixSphere(sphere_ix);
                }
                cloth_positions->push_back(node_position); // update cloth positions
                original_positions_.push_back(node_position);
                original_velocities_.push_back(node_velocity);
            }
        }


        
        for (int i = 0; i < cloth_dimensions_-1; i++){ // update cloth indices
            for (int j = 0; j < cloth_dimensions_-1; j++){
                int this_sphere = IndexOf(i,j);
                int sphere_right = IndexOf(i, j+1);
                int sphere_down = IndexOf(i+1, j);
                int sphere_down_right = IndexOf(i+1, j+1);

                cloth_indices->push_back(this_sphere);
                cloth_indices->push_back(sphere_down_right);
                cloth_indices->push_back(sphere_down);

                cloth_indices->push_back(this_sphere);
                cloth_indices->push_back(sphere_down_right);
                cloth_indices->push_back(sphere_right);
            }
        }
        patch_mesh_->UpdateIndices(std::move(cloth_indices)); 

        // update cloth normals
        std::vector<glm::vec3> normals; 
        std::vector<float> total_vertex_weight(positions_vec.size(), 0.0f);
        auto indices = patch_mesh_->GetIndices();
        for (size_t y = 0; y < indices.size()-2; y+=3){
            int A = indices[y];
            int B = indices[y+2];
            int C = indices[y+1];

            auto AB = positions_vec[A]-positions_vec[B];
            auto BC = positions_vec[C]-positions_vec[B];
            auto AB_BC = glm::cross(AB,BC);
            float AB_BC_length = glm::length(AB_BC);
            glm::vec3 face_normal = AB_BC/AB_BC_length;
            float face_normal_weight = AB_BC_length/2.0f;

            total_vertex_weight[A] = total_vertex_weight[A] + face_normal_weight;
            total_vertex_weight[B] = total_vertex_weight[B] + face_normal_weight;
            total_vertex_weight[C] = total_vertex_weight[C] + face_normal_weight;

            normals[A] = normals[A] + face_normal_weight * face_normal;
            normals[B] = normals[B] + face_normal_weight * face_normal;
            normals[C] = normals[C] + face_normal_weight * face_normal;
        } 

        for (size_t y = 0; y < normals.size(); y++){
            normals[y] = normals[y]/total_vertex_weight[y];
        }

        for (auto norm : normals){
            cloth_normals->push_back(norm);
        }

        patch_mesh_->UpdateNormals(std::move(cloth_normals));
        patch_mesh_->UpdatePositions(std::move(cloth_positions));


        
        std::unique_ptr<GLOO::SceneNode> patch_node = make_unique<SceneNode>(); /// display cloth
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
                    system_.AddSpring(this_sphere, sphere_right, 0.5, 0.5);
                }else if (j == cloth_dimensions_-1){
                    system_.AddSpring(this_sphere, sphere_down, 0.5, 0.5);
                }else{
                    system_.AddSpring(this_sphere, sphere_right, 0.5, 0.5);
                    system_.AddSpring(this_sphere, sphere_down, 0.5, 0.5);
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
        if (InputManager::GetInstance().IsKeyPressed('R')) {
            state_.positions = original_positions_;
            state_.velocities = original_velocities_;
        }


        auto cloth_positions = make_unique<PositionArray>();
        std::vector<glm::vec3> positions;
      auto new_state = integrator_->Integrate(system_, state_, time_, integration_step_);
      for (int i = 0; i < sphere_nodes_.size(); i++){
        sphere_nodes_[i]->GetTransform().SetPosition(new_state.positions[i]);
        cloth_positions->push_back(new_state.positions[i]);
        positions.push_back(new_state.positions[i]);
      }
      state_ = new_state;
      patch_mesh_->UpdatePositions(std::move(cloth_positions));


    // updating the normals
    auto indices = patch_mesh_->GetIndices();
    std::vector<glm::vec3> normals; 

    std::vector<float> total_vertex_weight(positions.size(), 0.0f);
    for (int i = 0; i < positions.size(); i++){
        normals.push_back(glm::vec3(0.0,0.0,0.0));
    }

    for (size_t y = 0; y < indices.size()-2; y+=3){
        int A = indices[y];
        int B = indices[y+2];
        int C = indices[y+1];

        auto AB = positions[A]-positions[B];
        auto BC = positions[C]-positions[B];
        auto AB_BC = glm::cross(AB,BC);
        float AB_BC_length = glm::length(AB_BC);
        glm::vec3 face_normal = AB_BC/AB_BC_length;
        float face_normal_weight = AB_BC_length/2.0f;

        total_vertex_weight[A] = total_vertex_weight[A] + face_normal_weight;
        total_vertex_weight[B] = total_vertex_weight[B] + face_normal_weight;
        total_vertex_weight[C] = total_vertex_weight[C] + face_normal_weight;


        normals[A] = normals[A] + face_normal_weight * face_normal;
        normals[B] = normals[B] + face_normal_weight * face_normal;
        normals[C] = normals[C] + face_normal_weight * face_normal;


    } 

    for (size_t y = 0; y < normals.size(); y++){
        normals[y] = normals[y]/total_vertex_weight[y];
    }

    auto norm_unique = make_unique<NormalArray>();
    for (auto norm : normals){
        norm_unique->push_back(norm);
    }

    patch_mesh_->UpdateNormals(std::move(norm_unique));


      time_ = time_ + delta_time;
    };


private:
    float integration_step_;
    std::vector<SceneNode*> sphere_nodes_;
    ParticleState state_;
    PendulumSystem system_;
    // PendulumSystem system_;

    std::vector<glm::vec3> original_positions_;
    std::vector<glm::vec3> original_velocities_;

    std::vector<std::vector<float>> spring_constants_;
    std::vector<std::vector<float>> spring_lengths_;
    std::vector<float> sphere_masses_;
    //  float drag_constant_ = 0.005;

    double time_;
    std::unique_ptr<IntegratorBase<TSystem, TState>> integrator_;
    IntegratorType integrator_type_;

    int cloth_dimensions_ = 16;
    std::shared_ptr<VertexObject> patch_mesh_ = std::make_shared<VertexObject>();
 
};

} // namespace GLOO

#endif