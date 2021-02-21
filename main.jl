# Modified by Ryan Lush for RMC ME699 Assignment 1 for use with "lush-robot.urdf"

# Defining function which will do the transformation and perform the simulation + animation

using RigidBodyDynamics
using LinearAlgebra
# using StaticArrays#, Plots
using MeshCat, MeshCatMechanisms, Blink
using GeometryTypes, CoordinateTransformations, ColorTypes

mvis, mechanism = display_urdf("lush-robot.urdf",vis)
state=MechanismState(mechanism)

function jacobian_transpose_ik!(state::MechanismState,
                               body::RigidBody,
                               point::Point3D,
                               desired::Point3D;
                               α=0.1,
                               iterations=100)
    # mechanism = state.mechanism
    world = root_frame(mechanism)

    # Compute the joint path from world to our target body
    p = path(mechanism, root_body(mechanism), body)
    # Allocate the point jacobian (we'll update this in-place later)
    Jp = point_jacobian(state, p, transform(state, point, world))

    q = copy(configuration(state))

    for i in 1:iterations
        # Update the position of the point
        point_in_world = transform(state, point, world)
        # Update the point's jacobian
        point_jacobian!(Jp, state, p, point_in_world)
        # Compute an update in joint coordinates using the jacobian transpose
        Δq = α * Array(Jp)' * (transform(state, desired, world) - point_in_world).v
        # Apply the update
        q .= configuration(state) .+ Δq
        set_configuration!(state, q)
    end
    state
end

function Assignment1(desiredpoint)
    jacobian_transpose_ik!(state,ee1body,ee1point,desiredpoint)
    set_configuration!(mvis, configuration(state))
end

#Setup

world_frame = root_frame(mechanism)

#grab the ee we want to control
ee1body = findbody(mechanism, "link6.1")
ee1frame = default_frame(ee1body)
ee1point = Point3D(ee1frame, 0.0,0.0,1.0)

setelement!(mvis,ee1point, 0.05)



# Obtain the transformation from world to ee

eeinBase = relative_transform(state,world_frame,ee1frame)
ee1point_in_world = inv(eeinBase)*ee1point
ee1point_in_ee1frame = eeinBase * ee1point_in_world

# print(ee1point_in_world)
# print(ee1point_in_ee1frame)

# User Input for Start and End point

print("Give coordinates with max value 3 for reach of robot\n")
print("give x of start point in world frame\n")

input1 = readline()
input1 = parse(Float64, input1)

print("give y of start point in world frame\n")

input2 = readline()
input2 = parse(Float64, input2)

print("give z of start point in world frame\n")

input3 = readline()
input3 = parse(Float64, input3)

x_start = Point3D(root_frame(mechanism), input1,input2,input3)

print("x_start = ", x_start)

print("Give coordinates with max value 3 for reach of robot\n")
print("give x of end point in world frame\n")

input4 = readline()
input4 = parse(Float64, input4)

print("give y of end point in world frame\n")

input5 = readline()
input5 = parse(Float64, input5)

print("give z of end point in world frame\n")

input6 = readline()
input6 = parse(Float64, input6)

x_end =  Point3D(root_frame(mechanism), input4,input5,input6)

Assignment1(x_start)

Assignment1(x_end)
