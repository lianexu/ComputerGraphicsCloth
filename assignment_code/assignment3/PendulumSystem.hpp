#ifndef PENDULUM_SYSTEM_H_
#define PENDULUM_SYSTEM_H_

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

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"


#include <fstream>

#include <string>
#include <vector>

#include "glm/ext.hpp" //for printing

namespace GLOO {
class PendulumSystem : public ParticleSystemBase {
  public:
    PendulumSystem(){};

   void AddSphere(float sphere_mass){
        if (sphere_masses_.size() == 0){ // we're adding the first mass [[0]]
            std::vector<float> row_init;
            row_init.push_back(0);
            spring_constants_.push_back(row_init);
            spring_lengths_.push_back(row_init);
        }else{
            std::vector<float> constants_row_init;
            std::vector<float> lengths_row_init;
            for (int i = 0; i < sphere_masses_.size(); i++){
                spring_constants_[i].push_back(0);
                spring_lengths_[i].push_back(0);
                constants_row_init.push_back(0);
                lengths_row_init.push_back(0);
            }
            constants_row_init.push_back(0);
            lengths_row_init.push_back(0);

            spring_constants_.push_back(constants_row_init);
            spring_lengths_.push_back(lengths_row_init);            
        }
        sphere_masses_.push_back(sphere_mass);
        fixed_spheres_.push_back(0);
   }


    void AddSpring(int sphere1, int sphere2, float spring_length, float spring_constant){ //connecting sphere1 and sphere2
        spring_constants_[sphere1][sphere2] = spring_constant;
        spring_constants_[sphere2][sphere1] = spring_constant;

        spring_lengths_[sphere1][sphere2] = spring_length;
        spring_lengths_[sphere2][sphere1] = spring_length;
    }
    

    void FixSphere(int sphere_index){ // spheres where fixed_spheres_[i] = 1 are fixed
        fixed_spheres_[sphere_index] = 1;
    }

    void SwitchWind(){
        // Turns wind on and off
        if (wind_is_on_){
            wind_is_on_ = false;
        }else{
            wind_is_on_ = true;
        }
        std::cout << "wind: " << wind_is_on_ << std::endl;
    }

    ParticleState ComputeTimeDerivative(const ParticleState& state, float time) const{
        // std::cout << "time: " << time << std::endl;
        ParticleState derivative;
        std::vector<glm::vec3> curr_positions = state.positions;
        std::vector<glm::vec3> velocities = state.velocities;
        std::vector<glm::vec3> accelerations;

        for(int i = 0; i < velocities.size(); i++){ // set the velocities of fixed spheres to 0
            if (sphere_masses_[i] == 0 || fixed_spheres_[i] == 1){
                glm::vec3 zero(0,0,0);
                velocities[i] = zero;
            }
         }

        std::vector<glm::vec3> gravity_forces;
        std::vector<glm::vec3> drag_forces;
        std::vector<glm::vec3> spring_forces;

        for(int i = 0; i < velocities.size(); i++){
            gravity_forces.push_back(glm::vec3(0.0,-9.8*sphere_masses_[i],0.0));
            drag_forces.push_back(-drag_constant_*velocities[i]);
            spring_forces.push_back(glm::vec3(0.0,0.0,0.0));
        }

         for(int i = 0; i < velocities.size(); i++){
             for(int j = 0; j < velocities.size(); j++){
                glm::vec3 i_pos = state.positions[i];
                glm::vec3 j_pos = state.positions[j];
                glm::vec3 ij = i_pos - j_pos;

                float ij_distance = glm::distance(i_pos,j_pos);
                float spring_length_ij = spring_lengths_[i][j]; // rest length
                float spring_constant_ij = spring_constants_[i][j];

                if (ij_distance != 0){
                    glm::vec3 spring_force_ij = -spring_constant_ij*(ij_distance-spring_length_ij)*(ij/ij_distance);
                    spring_forces[i] = spring_forces[i] + spring_force_ij;
                }
             }
         }
        int random_wind_y = -(rand() % 10 + 1); // random number from -1 to -10
        float random_wind_strength_z = -(float)rand()/(RAND_MAX + 1.0); //random number from 0 to -1
        float random_wind_strength_y = -(float)rand()/(RAND_MAX + 1.0); //random number from 0 to -1
        float random_wind_strength_x = -(float)rand()/(RAND_MAX + 1.0); //random number from 0 to -1
        glm::vec3 wind_strength(random_wind_strength_x,random_wind_strength_y,random_wind_strength_z);

         for(int i = 0; i < velocities.size(); i++){
            if (sphere_masses_[i] == 0 || fixed_spheres_[i] == 1){
                accelerations.push_back(glm::vec3(0,0,0));
            }else{
                if (curr_positions[i][1] > random_wind_y && wind_is_on_){ // if the y-coordinate of the node is less than random_wind_y, it is affected by the wind
                    accelerations.push_back(1/sphere_masses_[i] * (gravity_forces[i] + drag_forces[i] + spring_forces[i] + wind_strength));
                }else{
                    accelerations.push_back(1/sphere_masses_[i] * (gravity_forces[i] + drag_forces[i] + spring_forces[i]));
                }
            }
         }

        // putting it all together 
        derivative.positions = velocities;
        derivative.velocities = accelerations;
        return derivative;
    };

    private:
        std::vector<std::vector<float>> spring_constants_;
        std::vector<std::vector<float>> spring_lengths_;
        std::vector<float> sphere_masses_;
        std::vector<int> fixed_spheres_;
        float drag_constant_ = 0.005;
        int wind_is_on_ = true;
};
}  // namespace GLOO
#endif