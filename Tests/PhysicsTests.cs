using NUnit.Framework;
using ME.ECS;
using ME.ECS.Tests;
#if FIXED_POINT_MATH
using ME.ECS.Mathematics;
#else
using Unity.Mathematics;
using sfloat = System.Single;
#endif

namespace ME.ECS.Essentials.Physics.Tests {

    public class PhysicsTests {

        public struct TestComponent : IComponentBase {

            public int test;

        }

        public class TestSystem_AddRemove : ISystemBase, IAdvanceTick {
            
            public World world { get; set; }
            public void OnConstruct() {

                {
                    var body = new Entity("Static");
                    body.SetPosition(float3.zero);
                    body.SetRotation(quaternion.identity);
                    body.Set(new ME.ECS.Essentials.Physics.Components.PhysicsCollider() {
                        value = BoxCollider.Create(new BoxGeometry() {
                            BevelRadius = 0f,
                            Center = float3.zero,
                            Orientation = quaternion.identity,
                            Size = new float3(0f),
                        }),
                    });
                    body.Set<ME.ECS.Essentials.Physics.Components.IsPhysicsStatic>();
                }

                {
                    var body = new Entity("TestBody");
                    body.SetPosition(float3.zero);
                    body.SetRotation(quaternion.identity);
                    body.Set(new ME.ECS.Essentials.Physics.Components.PhysicsCollider() {
                        value = BoxCollider.Create(new BoxGeometry() {
                            BevelRadius = 0f,
                            Center = float3.zero,
                            Orientation = quaternion.identity,
                            Size = new float3(1f),
                        }),
                    });
                    body.Set(ME.ECS.Essentials.Physics.Components.PhysicsMass.CreateDynamic(MassProperties.UnitSphere, 1f));
                }

                {
                    var body = new Entity("TestBody2");
                    body.SetPosition(float3.zero);
                    body.SetRotation(quaternion.identity);
                    body.Set(new ME.ECS.Essentials.Physics.Components.PhysicsCollider() {
                        value = BoxCollider.Create(new BoxGeometry() {
                            BevelRadius = 0f,
                            Center = float3.zero,
                            Orientation = quaternion.identity,
                            Size = new float3(1f),
                        }),
                    });
                    body.Set(ME.ECS.Essentials.Physics.Components.PhysicsMass.CreateDynamic(MassProperties.UnitSphere, 1f));
                }

            }

            public void OnDeconstruct() {
                
            }

            public void AdvanceTick(in float deltaTime) {
                
                
                
            }

        }

        [Test]
        public void Test() {
            
            TestsHelper.Do((w) => {
                
                WorldUtilities.InitComponentTypeId<TestComponent>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsCollider>(false, true, true, isDisposable: true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsMass>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsInternal>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsOneShotInternal>(false, true, true, isOneShot: true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Transform.Container>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsCustomTags>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsVelocity>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsGravityFactor>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsDamping>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsMassOverride>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsJoint>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.PhysicsConstrainedBodyPair>(false, true, true);
                WorldUtilities.InitComponentTypeId<ME.ECS.Essentials.Physics.Components.IsPhysicsStatic>(true, true, false);

                ComponentsInitializerWorld.Setup((e) => {
                            
                    e.ValidateDataUnmanaged<TestComponent>();
                    e.ValidateDataUnmanagedDisposable<ME.ECS.Essentials.Physics.Components.PhysicsCollider>();
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsMass>();
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsInternal>();
                    e.ValidateDataOneShot<ME.ECS.Essentials.Physics.Components.PhysicsOneShotInternal>();
                    e.ValidateDataUnmanaged<ME.ECS.Transform.Container>();
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsCustomTags>();
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsVelocity>();
                    e.ValidateDataTag<ME.ECS.Essentials.Physics.Components.IsPhysicsStatic>(true);
                    
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsGravityFactor>();
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsDamping>();
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsMassOverride>();
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsJoint>();
                    e.ValidateDataUnmanaged<ME.ECS.Essentials.Physics.Components.PhysicsConstrainedBodyPair>();
                            
                });
                
            }, (w) => {
                
                w.AddFeature(new ME.ECS.Essentials.Physics.Core.UnityPhysicsFeature());

                var group = new SystemGroup(w, "TestGroup");
                group.AddSystem(new TestSystem_AddRemove());

            }, afterUpdate: (w) => {
                
                w.RemoveFeature(w.GetFeature<ME.ECS.Essentials.Physics.Core.UnityPhysicsFeature>());
                
            });

        }

    }

}