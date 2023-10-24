// My thing
// my stuff

// derived class of SceneNode
// override the Update Method to update the state and visualization
// keep your state, your integrator, and your physical system somewhere in this
// they need to be initialized in the contructor 

// update takes a float (elapsed time since the last frame). you will need to advance the state using
// your integrator for this amount of elapsed time for multiple steps.
// the number of integration steps depends on the step size passed in from the command line


#ifndef SIMPLE_SYSTEM_H_
#define SIMPLE_SYSTEM_H_

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

namespace GLOO {
class SimpleSystem : public ParticleSystemBase {
  public:
    SimpleSystem(){};

    ParticleState ComputeTimeDerivative(const ParticleState& state, float time) const{ //just const ParticleState& state?
        glm::vec3 position = state.positions[0]; //only one position
        glm::vec3 particle_derivative(-position[1],position[0],0);
        ParticleState derivative;
        derivative.positions.push_back(particle_derivative);
        derivative.velocities.push_back(glm::vec3(0,0,0));
        return derivative;
    };
};
}  // namespace GLOO
#endif