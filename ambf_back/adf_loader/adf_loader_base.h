//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef AF_ADF_LOADER_BASE_H
#define AF_ADF_LOADER_BASE_H
//------------------------------------------------------------------------------

#include "afAttributes.h"
#include <yaml-cpp/yaml.h>

using namespace ambf;

class ADFUtilsBase{
public:
    virtual bool getCartControllerAttribsFromNode(YAML::Node* a_node, afCartesianControllerAttributes* attribs){}

    virtual bool getCollisionAttribsFromNode(YAML::Node* a_node, afCollisionAttributes* attribs){}

    virtual bool getCommunicationAttribsFromNode(YAML::Node* a_node, afCommunicationAttributes* attribs){}

    virtual bool getHierarchyAttribsFromNode(YAML::Node* a_node, afHierarchyAttributes* attribs){}

    virtual bool getIdentificationAttribsFromNode(YAML::Node* a_node, afIdentificationAttributes* attribs){}

    virtual bool getInertialAttrisFromNode(YAML::Node* a_node, afInertialAttributes* attribs){}

    virtual bool getJointControllerAttribsFromNode(YAML::Node* a_node, afJointControllerAttributes* attribs){}

    virtual bool getKinematicAttribsFromNode(YAML::Node* a_node, afKinematicAttributes* attribs){}

    virtual bool getColorAttribsFromNode(YAML::Node* a_node, afColorAttributes* color){}

    virtual bool getShaderAttribsFromNode(YAML::Node* a_node, afShaderAttributes* attribs){}

    virtual bool getVisualAttribsFromNode(YAML::Node* a_node, afVisualAttributes* attribs){}

    virtual bool getSurfaceAttribsFromNode(YAML::Node* a_node, afSurfaceAttributes* attribs){}

    virtual bool getWheelAttribsFromNode(YAML::Node* a_node, afWheelAttributes* attribs){}

};
///
/// \brief The ADFLoader class
///
class ADFLoaderBase{
public:
    ADFLoaderBase(){}
    ~ADFLoaderBase(){}

    // Load object attributes
    virtual bool loadObjectAttribs(YAML::Node* a_node, std::string a_objName, afObjectType a_objType, afBaseObjectAttributes* a_objAttribs){}

    // Load Light Attributes
    virtual bool loadLightAttribs(YAML::Node* a_node, afLightAttributes* attribs){}

    // Load Camera Attributes
    virtual bool loadCameraAttribs(YAML::Node* a_node, afCameraAttributes* attribs){}

    // Load rigid body from a YAML::Node
    virtual bool loadRigidBodyAttribs(YAML::Node* a_node, afRigidBodyAttributes* attribs){}

    // Load soft body from a YAML::Node
    virtual bool loadSoftBodyAttribs(YAML::Node* a_node, afSoftBodyAttributes* attribs){}

    // Load ghost object from a YAML::Node
    virtual bool loadGhostObjectAttribs(YAML::Node* a_node, afGhostObjectAttributes* attribs){}

    // Load joint from a YAML::Node
    virtual bool loadJointAttribs(YAML::Node* a_node, afJointAttributes* attribs){}

    // Load joint from a YAML::Node
    virtual bool loadRayTracerSensorAttribs(YAML::Node* a_node, afRayTracerSensorAttributes* attribs){}

    // Load joint from a YAML::Node
    virtual bool loadResistanceSensorAttribs(YAML::Node* a_node, afResistanceSensorAttributes* attribs){}

    // Load actuator from a YAML::Node
    virtual bool loadActuatorAttribs(YAML::Node* a_node, afActuatorAttributes* attribs){}

    // Load actuator from a YAML::Node
    virtual bool loadConstraintActuatorAttribs(YAML::Node* a_node, afConstraintActuatorAttributes* attribs){}

    // Load sensor from a YAML::Node
    virtual bool loadVehicleAttribs(YAML::Node* a_node, afVehicleAttributes* attribs){}

    // Load Input Device Attributes
    virtual bool loadInputDeviceAttribs(YAML::Node* a_node, afInputDeviceAttributes *attribs){}

    // Load Simulated Device Attributes
    virtual bool loadSimulatedDeviceAttribs(YAML::Node* a_node, afSimulatedDeviceAttribs *attribs){}

    // Load all the TU device attributes
    virtual bool loadTeleRoboticUnitsAttribs(string a_filepath, vector<afTeleRoboticUnitAttributes>* attribs, vector<int> dev_indexes){}

    // Load model from ADF file
    virtual bool loadModelAttribs(string a_filepath, afModelAttributes* attribs){}

    // Load world from ADF file
    virtual bool loadWorldAttribs(string a_filepath, afWorldAttributes* attribs){}

    // Load the launch file
    virtual bool loadLaunchFileAttribs(string a_filepath, afLaunchAttributes* attribs){}

    // Get the version of this loader
    virtual std::string getLoaderVersion();

protected:
    std::string m_version = "NULL";
};
#endif
