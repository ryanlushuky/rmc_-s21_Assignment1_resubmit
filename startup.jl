# Modified by Ryan Lush for RMC ME699 Assignment 1 for use with "lush-robot.urdf"

import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics
using LinearAlgebra
# using StaticArrays#, Plots
using MeshCat, MeshCatMechanisms, Blink
using GeometryTypes, CoordinateTransformations, ColorTypes
vis = Visualizer();open(vis)


function display_urdf(urdfPath,vis)
    # Displays mechanism at config all zeros
    # urdfPath must be a string
    #urdfPath = "lush-robot.urdf"

    mechanism = parse_urdf(Float64,urdfPath)

    state = MechanismState(mechanism)
    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    for bd in bodies(mechanism)
       setelement!(mvis,default_frame(bd),1,"$bd")
    end
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    return mvis, mechanism
end

mvis, mechanism = display_urdf("lush-robot.urdf",vis)
state=MechanismState(mechanism)
set_configuration!(state, [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# First four angles are joint up to but not including the branch
# Angles 5 & 6 are each of the branch Angles
# Last 2 angles are the angles of the arm with the end effector at the end
set_configuration!(mvis, configuration(state))
