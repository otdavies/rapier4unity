mod handles;
mod utils;
use crate::handles::{
    SerializableColliderHandle, SerializableRigidBodyHandle, SerializableRigidBodyType,
};
use handles::SerializableImpulseJointHandle;
use rapier3d::crossbeam;
use rapier3d::na::{Isometry, Quaternion, UnitQuaternion, Vector2, Vector3, Vector4};
use rapier3d::prelude::*;
use std::mem;
use unitybridge::{AssignUnityLogger, IUnityLog};
use utils::{
    cancel_axis_velocity, locked_axes_to_unity_constraints, unity_constraints_to_locked_axes,
};

static mut PHYSIC_SOLVER_DATA: Option<PhysicsSolverData> = None;

#[allow(static_mut_refs)]
fn get_mutable_physics_solver() -> &'static mut PhysicsSolverData<'static> {
    unsafe { PHYSIC_SOLVER_DATA.as_mut().unwrap() }
}

#[repr(C)]
struct FunctionsToCallFromRust {
    unity_log_ptr: *const IUnityLog,
}

#[unsafe(no_mangle)]
extern "C" fn init(funcs: *const FunctionsToCallFromRust) {
    unsafe {
        PHYSIC_SOLVER_DATA = Some(PhysicsSolverData::default());
        AssignUnityLogger((*funcs).unity_log_ptr);
    }
}

#[unsafe(no_mangle)]
extern "C" fn hello_world() {
    log::info!("Hello, cake!");
}

// teardown
#[unsafe(no_mangle)]
extern "C" fn teardown() {
    unsafe {
        PHYSIC_SOLVER_DATA = None;
    }
}

#[repr(C)]
struct RawArray<T> {
    ptr: *mut T,
    len: usize,
    capacity: usize,
}

#[unsafe(no_mangle)]
#[allow(static_mut_refs)]
extern "C" fn solve() -> *const RawArray<SerializableCollisionEvent> {
    unsafe {
        if PHYSIC_SOLVER_DATA.is_none() {
            log::warn!("Physics solver data is not initialized");
            return std::ptr::null();
        }
    }

    let mut collision_events = get_mutable_physics_solver().solve();
    // box the vector to prevent it from being deallocated
    let ptr = collision_events.as_mut_ptr();
    let len = collision_events.len();
    let capacity = collision_events.capacity();
    let val = Box::new(RawArray { ptr, len, capacity });
    mem::forget(collision_events);
    Box::into_raw(val)
}

#[unsafe(no_mangle)]
extern "C" fn free_collision_events(ptr: *mut RawArray<SerializableCollisionEvent>) {
    unsafe {
        let info = Box::from_raw(ptr);
        let _ = Vec::from_raw_parts(info.ptr, info.len, info.capacity);
    }
}

// Settings

#[unsafe(no_mangle)]
extern "C" fn set_gravity(x: f32, y: f32, z: f32) {
    get_mutable_physics_solver().gravity = vector![x, y, z];
}

#[unsafe(no_mangle)]
extern "C" fn set_time_step(dt: f32) {
    get_mutable_physics_solver().integration_parameters.dt = dt;
    get_mutable_physics_solver()
        .integration_parameters
        .min_ccd_dt = dt / 100.0;
}

// Collider

#[unsafe(no_mangle)]
extern "C" fn add_cuboid_collider(
    half_extents_x: f32,
    half_extents_y: f32,
    half_extents_z: f32,
    mass: f32,
    is_sensor: bool,
) -> SerializableColliderHandle {
    let psd = get_mutable_physics_solver();
    let collider = ColliderBuilder::cuboid(half_extents_x, half_extents_y, half_extents_z)
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .density(mass)
        .sensor(is_sensor)
        .build();
    psd.collider_set.insert(collider).into()
}

#[unsafe(no_mangle)]
extern "C" fn add_sphere_collider(
    radius: f32,
    mass: f32,
    is_sensor: bool,
) -> SerializableColliderHandle {
    let psd = get_mutable_physics_solver();
    let collider = ColliderBuilder::ball(radius)
        .density(mass)
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .sensor(is_sensor)
        .build();
    psd.collider_set.insert(collider).into()
}

#[unsafe(no_mangle)]
extern "C" fn add_capsule_collider(
    half_height: f32,
    radius: f32,
    mass: f32,
    is_sensor: bool,
) -> SerializableColliderHandle {
    let psd = get_mutable_physics_solver();
    let collider = ColliderBuilder::capsule_y(half_height, radius)
        .density(mass)
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .sensor(is_sensor)
        .build();
    psd.collider_set.insert(collider).into()
}

// TODO Investigate optimizing this a bit
#[unsafe(no_mangle)]
extern "C" fn add_mesh_collider(
    vertices_ptr: *const f32,
    vertices_count: usize,
    indices_ptr: *const u32,
    indices_count: usize,
    mass: f32,
    is_sensor: bool,
) -> SerializableColliderHandle {
    let psd = get_mutable_physics_solver();

    // Convert C arrays to Rust slices
    let vertices_flat = unsafe { std::slice::from_raw_parts(vertices_ptr, vertices_count * 3) };
    let indices_flat = unsafe { std::slice::from_raw_parts(indices_ptr, indices_count * 3) };

    // Convert flat arrays to points
    let mut vertices = Vec::with_capacity(vertices_count);
    for i in 0..vertices_count {
        vertices.push(point![
            vertices_flat[i * 3],
            vertices_flat[i * 3 + 1],
            vertices_flat[i * 3 + 2]
        ]);
    }

    // Convert flat indices to triangle indices
    let mut indices = Vec::with_capacity(indices_count);
    for i in 0..indices_count {
        indices.push([
            indices_flat[i * 3],
            indices_flat[i * 3 + 1],
            indices_flat[i * 3 + 2],
        ]);
    }

    // Build the trimesh collider
    if let Ok(collider_builder) = ColliderBuilder::trimesh(vertices, indices) {
        let collider = collider_builder
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .density(mass)
            .sensor(is_sensor)
            .build();
        psd.collider_set.insert(collider).into()
    } else {
        log::warn!("Failed to create mesh collider");
        ColliderHandle::invalid().into()
    }
}

// TODO Investigate optimizing this a bit
// In practice we may only want to use this option as opposed to full mesh collision
#[unsafe(no_mangle)]
extern "C" fn add_convex_mesh_collider(
    vertices_ptr: *const f32,
    vertices_count: usize,
    mass: f32,
    is_sensor: bool,
) -> SerializableColliderHandle {
    let psd = get_mutable_physics_solver();

    // Convert C arrays to Rust slices
    let vertices_flat = unsafe { std::slice::from_raw_parts(vertices_ptr, vertices_count * 3) };

    // Convert flat arrays to points
    let mut points = Vec::with_capacity(vertices_count);
    for i in 0..vertices_count {
        points.push(point![
            vertices_flat[i * 3],
            vertices_flat[i * 3 + 1],
            vertices_flat[i * 3 + 2]
        ]);
    }

    // Build the convex hull collider
    if let Some(collider_builder) = ColliderBuilder::convex_hull(&points) {
        let collider = collider_builder
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .density(mass)
            .sensor(is_sensor)
            .build();
        psd.collider_set.insert(collider).into()
    } else {
        log::warn!("Failed to create convex hull collider");
        ColliderHandle::invalid().into()
    }
}

// RigidBody

#[unsafe(no_mangle)]
extern "C" fn add_rigid_body(
    collider: SerializableColliderHandle,
    rb_type: SerializableRigidBodyType,
    position_x: f32,
    position_y: f32,
    position_z: f32,
    rotation_x: f32,
    rotation_y: f32,
    rotation_z: f32,
    rotation_w: f32,
) -> SerializableRigidBodyHandle {
    let psd = get_mutable_physics_solver();
    let quat = Quaternion::new(rotation_w, rotation_x, rotation_y, rotation_z);

    // Convert to unit quaternion
    let unit_quat = UnitQuaternion::from_quaternion(quat);

    // Extract the rotation angle and axis
    let angle = unit_quat.angle();
    let axis = unit_quat.axis().unwrap_or(Vector3::z_axis());

    // Create the AngVector (axis-angle representation)
    let ang_vector = axis.into_inner() * angle;

    // Build with the AngVector
    let rigid_body = RigidBodyBuilder::new(rb_type.into())
        .translation(vector![position_x, position_y, position_z])
        .rotation(ang_vector)
        .build();

    let rb_handle = psd.rigid_body_set.insert(rigid_body);
    psd.collider_set
        .set_parent(collider.into(), Some(rb_handle), &mut psd.rigid_body_set);
    rb_handle.into()
}

#[unsafe(no_mangle)]
extern "C" fn remove_rigid_body(rb_handle: SerializableRigidBodyHandle) {
    let psd = get_mutable_physics_solver();
    psd.rigid_body_set.remove(
        rb_handle.into(),
        &mut psd.island_manager,
        &mut psd.collider_set,
        &mut psd.impulse_joint_set,
        &mut psd.multibody_joint_set,
        true,
    );
}

#[unsafe(no_mangle)]
extern "C" fn update_rigid_body_properties(
    rb_handle: SerializableRigidBodyHandle,
    rb_type: SerializableRigidBodyType,
    enable_ccd: bool,
    constraints: u32,
    linear_drag: f32,
    angular_drag: f32,
) {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get_mut(rb_handle.into()).unwrap();

    // Update body type if different
    let rb_type_enum: RigidBodyType = rb_type.into();
    if rb.body_type() != rb_type_enum {
        rb.set_body_type(rb_type_enum, true);
    }

    // Set CCD
    rb.enable_ccd(enable_ccd);

    // Update constraints if different
    if locked_axes_to_unity_constraints(rb.locked_axes()) != constraints {
        let locks = unity_constraints_to_locked_axes(constraints);
        rb.set_locked_axes(locks, false);
        cancel_axis_velocity(locks, rb);
    }

    // Set drag values
    rb.set_linear_damping(linear_drag);
    rb.set_angular_damping(angular_drag);
}

#[unsafe(no_mangle)]
extern "C" fn add_fixed_joint(
    rb1_handle: SerializableRigidBodyHandle,
    rb2_handle: SerializableRigidBodyHandle,
    local_frame1_x: f32,
    local_frame1_y: f32,
    local_frame1_z: f32,
    local_frame2_x: f32,
    local_frame2_y: f32,
    local_frame2_z: f32,
    self_collision: bool,
) -> SerializableImpulseJointHandle {
    let psd = get_mutable_physics_solver();
    let point1: Point<Real> = point![local_frame1_x, local_frame1_y, local_frame1_z];
    let point2: Point<Real> = point![local_frame2_x, local_frame2_y, local_frame2_z];
    let anchor_rb = psd.rigid_body_set.get(rb1_handle.into()).unwrap();
    let mover_rb = psd.rigid_body_set.get(rb2_handle.into()).unwrap();
    // Construct the local_frame for the first body
    let local_frame1 =
        Isometry::from_parts(Translation::from(point1), anchor_rb.position().rotation);
    // Construct the local_frame for the second body
    let local_frame2 =
        Isometry::from_parts(Translation::from(point2), mover_rb.position().rotation);
    let joint = FixedJointBuilder::new()
        .local_frame1(local_frame1)
        .local_frame2(local_frame2)
        .contacts_enabled(self_collision);

    psd.impulse_joint_set
        .insert(rb1_handle.into(), rb2_handle.into(), joint, false)
        .into()
}

#[unsafe(no_mangle)]
extern "C" fn add_spherical_joint(
    rb1_handle: SerializableRigidBodyHandle,
    rb2_handle: SerializableRigidBodyHandle,
    local_frame1_x: f32,
    local_frame1_y: f32,
    local_frame1_z: f32,
    local_frame2_x: f32,
    local_frame2_y: f32,
    local_frame2_z: f32,
    self_collision: bool,
) -> SerializableImpulseJointHandle {
    let psd = get_mutable_physics_solver();
    let point1: Point<Real> = point![local_frame1_x, local_frame1_y, local_frame1_z];
    let point2: Point<Real> = point![local_frame2_x, local_frame2_y, local_frame2_z];
    let anchor_rb = psd.rigid_body_set.get(rb1_handle.into()).unwrap();
    let mover_rb = psd.rigid_body_set.get(rb2_handle.into()).unwrap();
    // Construct the local_frame for the first body
    let local_frame1 =
        Isometry::from_parts(Translation::from(point1), anchor_rb.position().rotation);
    // Construct the local_frame for the second body
    let local_frame2 =
        Isometry::from_parts(Translation::from(point2), mover_rb.position().rotation);
    let joint = SphericalJointBuilder::new()
        // .local_anchor1(point1)
        // .local_anchor2(point2)
        .local_frame1(local_frame1)
        .local_frame2(local_frame2)
        .contacts_enabled(self_collision);

    psd.impulse_joint_set
        .insert(rb1_handle.into(), rb2_handle.into(), joint, false)
        .into()
}

#[unsafe(no_mangle)]
extern "C" fn add_revolute_joint(
    rb1_handle: SerializableRigidBodyHandle,
    rb2_handle: SerializableRigidBodyHandle,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    local_frame1_x: f32,
    local_frame1_y: f32,
    local_frame1_z: f32,
    local_frame2_x: f32,
    local_frame2_y: f32,
    local_frame2_z: f32,
    self_collision: bool,
) -> SerializableImpulseJointHandle {
    let psd = get_mutable_physics_solver();
    let point1: Point<Real> = point![local_frame1_x, local_frame1_y, local_frame1_z];
    let point2: Point<Real> = point![local_frame2_x, local_frame2_y, local_frame2_z];
    let axis: UnitVector<Real> = UnitVector::new_normalize(vector![axis_x, axis_y, axis_z]);
    let joint = RevoluteJointBuilder::new(axis)
        .local_anchor1(point1)
        .local_anchor2(point2)
        .contacts_enabled(self_collision);

    psd.impulse_joint_set
        .insert(rb1_handle.into(), rb2_handle.into(), joint, false)
        .into()
}

#[unsafe(no_mangle)]
extern "C" fn add_prismatic_joint(
    rb1_handle: SerializableRigidBodyHandle,
    rb2_handle: SerializableRigidBodyHandle,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    local_frame1_x: f32,
    local_frame1_y: f32,
    local_frame1_z: f32,
    local_frame2_x: f32,
    local_frame2_y: f32,
    local_frame2_z: f32,
    limit_min: f32,
    limit_max: f32,
    self_collision: bool,
) -> SerializableImpulseJointHandle {
    let psd = get_mutable_physics_solver();
    let point1: Point<Real> = point![local_frame1_x, local_frame1_y, local_frame1_z];
    let point2: Point<Real> = point![local_frame2_x, local_frame2_y, local_frame2_z];
    let axis: UnitVector<Real> = UnitVector::new_normalize(vector![axis_x, axis_y, axis_z]);
    let joint = PrismaticJointBuilder::new(axis)
        .local_anchor1(point1)
        .local_anchor2(point2)
        .limits([limit_min, limit_max])
        // If the anchor is kinematic, then don't collide with the other body
        .contacts_enabled(self_collision);

    psd.impulse_joint_set
        .insert(rb1_handle.into(), rb2_handle.into(), joint, false)
        .into()
}

#[unsafe(no_mangle)]
extern "C" fn remove_joint(handle: SerializableImpulseJointHandle) {
    let psd = get_mutable_physics_solver();
    psd.impulse_joint_set.remove(handle.into(), true);
}

#[unsafe(no_mangle)]
extern "C" fn get_transform(rb_handle: SerializableRigidBodyHandle) -> RapierTransform {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get(rb_handle.into()).unwrap();
    let pos = rb.position();
    RapierTransform {
        rotation: pos.rotation.coords,
        position: pos.translation.vector,
    }
}

#[unsafe(no_mangle)]
extern "C" fn set_transform_position(
    rb_handle: SerializableRigidBodyHandle,
    position_x: f32,
    position_y: f32,
    position_z: f32,
) {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get_mut(rb_handle.into()).unwrap();
    let iso = Isometry::from_parts(
        Translation::from(vector![position_x, position_y, position_z]),
        // This is a trick to make sure the order of the position versus rotation will work in either order
        rb.next_position().rotation,
    );
    rb.set_next_kinematic_position(iso);
}

#[unsafe(no_mangle)]
extern "C" fn set_transform_rotation(
    rb_handle: SerializableRigidBodyHandle,
    rotation_x: f32,
    rotation_y: f32,
    rotation_z: f32,
    rotation_w: f32,
) {
    let psd = get_mutable_physics_solver();
    let rb: &mut RigidBody = psd.rigid_body_set.get_mut(rb_handle.into()).unwrap();
    rb.set_next_kinematic_rotation(UnitQuaternion::new_normalize(Quaternion::new(
        rotation_w, rotation_x, rotation_y, rotation_z,
    )));
}

#[unsafe(no_mangle)]
extern "C" fn set_transform(
    rb_handle: SerializableRigidBodyHandle,
    position_x: f32,
    position_y: f32,
    position_z: f32,
    rotation_x: f32,
    rotation_y: f32,
    rotation_z: f32,
    rotation_w: f32,
) {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get_mut(rb_handle.into()).unwrap();
    let iso = Isometry::from_parts(
        Translation::from(vector![position_x, position_y, position_z]),
        UnitQuaternion::new_normalize(Quaternion::new(
            rotation_w, rotation_x, rotation_y, rotation_z,
        )),
    );
    rb.set_next_kinematic_position(iso);
}

#[unsafe(no_mangle)]
extern "C" fn set_linear_velocity(
    rb_handle: SerializableRigidBodyHandle,
    velocity_x: f32,
    velocity_y: f32,
    velocity_z: f32,
) {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get_mut(rb_handle.into()).unwrap();
    rb.set_linvel(vector![velocity_x, velocity_y, velocity_z], true);
}

#[unsafe(no_mangle)]
extern "C" fn set_angular_velocity(
    rb_handle: SerializableRigidBodyHandle,
    velocity_x: f32,
    velocity_y: f32,
    velocity_z: f32,
) {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get_mut(rb_handle.into()).unwrap();
    rb.set_angvel(vector![velocity_x, velocity_y, velocity_z], true);
}

#[unsafe(no_mangle)]
extern "C" fn get_linear_velocity(rb_handle: SerializableRigidBodyHandle) -> Vector3<f32> {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get(rb_handle.into()).unwrap();
    rb.linvel().clone()
}

#[unsafe(no_mangle)]
extern "C" fn get_angular_velocity(rb_handle: SerializableRigidBodyHandle) -> Vector3<f32> {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get(rb_handle.into()).unwrap();
    rb.angvel().clone()
}

// Add Force
#[unsafe(no_mangle)]
extern "C" fn add_force(
    rb_handle: SerializableRigidBodyHandle,
    force_x: f32,
    force_y: f32,
    force_z: f32,
    mode: ForceMode,
) {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get_mut(rb_handle.into()).unwrap();
    let mut linvel = rb.linvel().clone();
    match mode {
        ForceMode::Force => {
            linvel +=
                vector![force_x, force_y, force_z] * psd.integration_parameters.dt / rb.mass();
        }
        ForceMode::Impulse => {
            linvel += vector![force_x, force_y, force_z] / rb.mass();
        }
        ForceMode::VelocityChange => {
            linvel += vector![force_x, force_y, force_z];
        }
        ForceMode::Acceleration => {
            linvel += vector![force_x, force_y, force_z] * psd.integration_parameters.dt;
        }
    }
    // log::info!("linvel: {:?}, mode: {:?}", linvel, mode);
    rb.set_linvel(linvel, true);
}

#[unsafe(no_mangle)]
extern "C" fn add_torque(
    rb_handle: SerializableRigidBodyHandle,
    torque_x: f32,
    torque_y: f32,
    torque_z: f32,
    mode: ForceMode,
) {
    let psd = get_mutable_physics_solver();
    let rb = psd.rigid_body_set.get_mut(rb_handle.into()).unwrap();
    let mut angvel = rb.angvel().clone();
    match mode {
        ForceMode::Force => {
            angvel +=
                vector![torque_x, torque_y, torque_z] * psd.integration_parameters.dt / rb.mass();
        }
        ForceMode::Impulse => {
            angvel += vector![torque_x, torque_y, torque_z] / rb.mass();
        }
        ForceMode::VelocityChange => {
            angvel += vector![torque_x, torque_y, torque_z];
        }
        ForceMode::Acceleration => {
            angvel += vector![torque_x, torque_y, torque_z] * psd.integration_parameters.dt;
        }
    }
    rb.set_angvel(angvel, true);
}

#[unsafe(no_mangle)]
pub extern "C" fn set_integration_parameters(
    // Time step
    dt: f32,
    // Solver parameters
    solver_iterations: usize,
    solver_pgs_iterations: usize,
    solver_additional_friction_iterations: usize,
    solver_stabilization_iterations: usize,
    ccd_substeps: usize,
    // Damping parameters
    contact_damping_ratio: f32,
    joint_damping_ratio: f32,
    // Frequency parameters
    contact_frequency: f32,
    joint_frequency: f32,
    // Prediction parameters
    prediction_distance: f32,
    max_corrective_velocity: f32,
    // Length unit
    length_unit: f32,
) {
    use std::num::NonZeroUsize;

    let psd = get_mutable_physics_solver();
    psd.integration_parameters.dt = dt;
    psd.integration_parameters.min_ccd_dt = dt / 100.0;
    psd.integration_parameters.num_solver_iterations =
        NonZeroUsize::new(solver_iterations as usize).unwrap_or(NonZeroUsize::new(4).unwrap());
    psd.integration_parameters.num_internal_pgs_iterations = solver_pgs_iterations;
    psd.integration_parameters
        .num_additional_friction_iterations = solver_additional_friction_iterations;
    psd.integration_parameters
        .num_internal_stabilization_iterations = solver_stabilization_iterations;
    psd.integration_parameters.max_ccd_substeps = ccd_substeps;
    psd.integration_parameters.contact_damping_ratio = contact_damping_ratio;
    psd.integration_parameters.joint_damping_ratio = joint_damping_ratio;
    psd.integration_parameters.contact_natural_frequency = contact_frequency;
    psd.integration_parameters.joint_natural_frequency = joint_frequency;
    psd.integration_parameters.normalized_prediction_distance = prediction_distance;
    psd.integration_parameters
        .normalized_max_corrective_velocity = max_corrective_velocity;
    psd.integration_parameters.length_unit = length_unit;
}

// Scene Query
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct RaycastHit {
    m_point: Vector3<f32>,
    m_normal: Vector3<f32>,
    m_face_id: u32,
    m_distance: f32,
    m_uv: Vector2<f32>,
    m_collider: SerializableColliderHandle,
}

#[unsafe(no_mangle)]
extern "C" fn cast_ray(
    from_x: f32,
    from_y: f32,
    from_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    out_hit: *mut RaycastHit,
) -> bool {
    let psd = get_mutable_physics_solver();
    let ray = Ray::new(point![from_x, from_y, from_z], vector![dir_x, dir_y, dir_z]);
    if let Some((handle, intersection)) = psd.query_pipeline.cast_ray_and_get_normal(
        &psd.rigid_body_set,
        &psd.collider_set,
        &ray,
        4.0,
        true,
        QueryFilter::default(),
    ) {
        let point = ray.point_at(intersection.time_of_impact);
        let normal = intersection.normal;
        let face_id = match intersection.feature {
            FeatureId::Face(id) => id,
            FeatureId::Vertex(id) => id,
            FeatureId::Edge(id) => id,
            _ => 0,
        };
        let distance = intersection.time_of_impact;
        let uv = vector![0.0, 0.0];
        let hit = RaycastHit {
            m_point: point.coords,
            m_normal: normal,
            m_face_id: face_id,
            m_distance: distance,
            m_uv: uv,
            m_collider: handle.into(),
        };
        unsafe {
            *out_hit = hit;
        }
        true
    } else {
        false
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub enum ForceMode {
    Force = 0,
    Impulse = 1,
    VelocityChange = 2,
    Acceleration = 5,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct RapierTransform {
    rotation: Vector4<f32>,
    position: Vector<f32>,
}

// PhysicsSolverData is a struct that holds all the data needed to solve physics.
pub struct PhysicsSolverData<'a> {
    pub gravity: Vector<f32>,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub physics_hooks: &'a dyn PhysicsHooks,
    pub event_handler: &'a dyn EventHandler,

    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
}

impl Default for PhysicsSolverData<'_> {
    fn default() -> Self {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 50.0;
        integration_parameters.min_ccd_dt = 1.0 / 50.0 / 100.0;
        PhysicsSolverData {
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            physics_hooks: &(),
            event_handler: &(),

            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct SerializableCollisionEvent {
    collider1: SerializableColliderHandle,
    collider2: SerializableColliderHandle,
    is_started: bool,
}

impl PhysicsSolverData<'_> {
    fn solve(&mut self) -> Vec<SerializableCollisionEvent> {
        let (collision_send, collision_recv) = crossbeam::channel::unbounded();
        let (contact_force_send, _contact_force_recv) = crossbeam::channel::unbounded();
        let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &event_handler,
        );

        let mut collision_events = Vec::new();
        while let Ok(collision_event) = collision_recv.try_recv() {
            if collision_event.started() {
                collision_events.push(SerializableCollisionEvent {
                    collider1: collision_event.collider1().into(),
                    collider2: collision_event.collider2().into(),
                    is_started: true,
                });
            } else if collision_event.stopped() {
                collision_events.push(SerializableCollisionEvent {
                    collider1: collision_event.collider1().into(),
                    collider2: collision_event.collider2().into(),
                    is_started: false,
                });
            } else {
                log::warn!("Unknown collision event: {:?}", collision_event);
            }
        }

        collision_events
    }
}
