using ME.ECS;
#if FIXED_POINT_MATH
using ME.ECS.Mathematics;
#else
using Unity.Mathematics;
using tfloat = System.Single;
#endif

namespace ME.ECS.Essentials.Physics.Components {

    public struct PhysicsInternal : IComponent {

        public int prevStaticCount;

    }

    public struct PhysicsOneShotInternal : IComponentOneShot {

        public ME.ECS.Essentials.Physics.CollisionEvents collisionEvents;
        public ME.ECS.Essentials.Physics.TriggerEvents triggerEvents;

    }

    public struct IsPhysicsStatic : IComponent, IComponentBlittable {
    }

    public struct PhysicsCustomTags : IComponent {

        public byte value;

    }

    public struct PhysicsCollider : IComponent, IComponentDisposable<PhysicsCollider>, ICopyableUnmanaged {
        
        public BlobAssetReference<ME.ECS.Essentials.Physics.Collider> value;  // null is allowed

        public bool IsValid => this.value.IsCreated;
        public unsafe ME.ECS.Essentials.Physics.Collider* ColliderPtr => (ME.ECS.Essentials.Physics.Collider*)this.value.GetUnsafePtr();
        public ME.ECS.Essentials.Physics.MassProperties MassProperties => this.value.IsCreated ? this.value.Value.MassProperties : ME.ECS.Essentials.Physics.MassProperties.UnitSphere;

        public void ReplaceWith(ref ME.ECS.Collections.LowLevel.Unsafe.MemoryAllocator allocator, in PhysicsCollider other) {

            if (this.value.IsCreated == true && other.value.IsCreated == true) {

                this.value.Dispose();
                this.value = other.value.Value.Clone();
                
            } else if (this.value.IsCreated == true && other.value.IsCreated == false) {
                
                this.value.Dispose();
                this.value = default;

            } else if (this.value.IsCreated == false && other.value.IsCreated == true) {
                
                this.value = other.value.Value.Clone();

            } else if (this.value.IsCreated == false && other.value.IsCreated == false) {
                
                // Nothing to do
                
            }
            
        }

        public void OnDispose(ref ME.ECS.Collections.LowLevel.Unsafe.MemoryAllocator allocator) {

            if (this.value.IsCreated == true && this.value.IsValid == true) {
                this.value.Dispose();
            }
            this.value = default;

        }

    }
    
    public struct PhysicsConstrains : IComponent {
        
        public float3 position;
        public float3 rotation;
        
    }
    
    public struct PhysicsVelocity : IComponent {

        public float3 Linear;
        public float3 Angular;

    }

    public struct PhysicsDamping : IComponent {

        public tfloat Linear;
        public tfloat Angular;

    }
    
    public struct PhysicsMassOverride : IComponent {
        
        public byte IsKinematic;
        
    }

    public struct PhysicsMass : IComponent {

        public RigidTransform Transform;
        public tfloat InverseMass;
        public float3 InverseInertia;
        public tfloat AngularExpansionFactor;
        
        public float3 CenterOfMass { get => Transform.pos; set => Transform.pos = value; }
        public quaternion InertiaOrientation { get => Transform.rot; set => Transform.rot = value; }

        public static PhysicsMass CreateDynamic(ME.ECS.Essentials.Physics.MassProperties massProperties, tfloat mass)
        {
            //SafetyChecks.CheckFiniteAndPositiveAndThrow(mass, nameof(mass));

            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = math.rcp(mass),
                InverseInertia = math.rcp(massProperties.MassDistribution.InertiaTensor * mass),
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }

        public static PhysicsMass CreateKinematic(ME.ECS.Essentials.Physics.MassProperties massProperties)
        {
            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = 0,
                InverseInertia = float3.zero,
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }

    }

    public struct PhysicsGravityFactor : IComponent {

        public tfloat value;

    }

}