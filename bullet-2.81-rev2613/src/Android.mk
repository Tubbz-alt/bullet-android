CFLAGS = -O2 -DTARGET_FLOAT32_IS_FIXED -IInclude -Iandroid

TARGETS = android

LIB = libBulletCollision.a

SOURCES = \
./BulletSoftBody/btDefaultSoftBodySolver.cpp \
./BulletSoftBody/btSoftRigidDynamicsWorld.cpp \
./BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp \
./BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp \
./BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp \
./BulletSoftBody/btSoftBody.cpp \
./BulletSoftBody/btSoftBodyHelpers.cpp \
./BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp \
./LinearMath/btSerializer.cpp \
./LinearMath/btVector3.cpp \
./LinearMath/btPolarDecomposition.cpp \
./LinearMath/btConvexHullComputer.cpp \
./LinearMath/btAlignedAllocator.cpp \
./LinearMath/btGeometryUtil.cpp \
./LinearMath/btQuickprof.cpp \
./LinearMath/btConvexHull.cpp \
./BulletMultiThreaded/PosixThreadSupport.cpp \
./BulletMultiThreaded/Win32ThreadSupport.cpp \
./BulletMultiThreaded/SpuSampleTaskProcess.cpp \
./BulletMultiThreaded/btThreadSupportInterface.cpp \
./BulletMultiThreaded/SpuLibspe2Support.cpp \
./BulletMultiThreaded/SpuFakeDma.cpp \
./BulletMultiThreaded/GpuSoftBodySolvers/DX11/btSoftBodySolver_DX11.cpp \
./BulletMultiThreaded/GpuSoftBodySolvers/DX11/btSoftBodySolver_DX11SIMDAware.cpp \
./BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolverOutputCLtoGL.cpp \
./BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolver_OpenCL.cpp \
./BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolver_OpenCLSIMDAware.cpp \
./BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/MiniCLTaskWrap.cpp \
./BulletMultiThreaded/SpuCollisionTaskProcess.cpp \
./BulletMultiThreaded/SpuSampleTask/SpuSampleTask.cpp \
./BulletMultiThreaded/btParallelConstraintSolver.cpp \
./BulletMultiThreaded/SequentialThreadSupport.cpp \
./BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.cpp \
./BulletMultiThreaded/btGpu3DGridBroadphase.cpp \
./BulletMultiThreaded/SpuGatheringCollisionDispatcher.cpp \
./BulletMultiThreaded/SpuNarrowPhaseCollisionTask/boxBoxDistance.cpp \
./BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuContactResult.cpp \
./BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.cpp \
./BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.cpp \
./BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuCollisionShapes.cpp \
./BulletMultiThreaded/SpuCollisionObjectWrapper.cpp \
./BulletDynamics/ConstraintSolver/btTypedConstraint.cpp \
./BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp \
./BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp \
./BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp \
./BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp \
./BulletDynamics/ConstraintSolver/btContactConstraint.cpp \
./BulletDynamics/ConstraintSolver/btHingeConstraint.cpp \
./BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp \
./BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp \
./BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp \
./BulletDynamics/ConstraintSolver/btGearConstraint.cpp \
./BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp \
./BulletDynamics/ConstraintSolver/btSliderConstraint.cpp \
./BulletDynamics/Vehicle/btRaycastVehicle.cpp \
./BulletDynamics/Vehicle/btWheelInfo.cpp \
./BulletDynamics/Character/btKinematicCharacterController.cpp \
./BulletDynamics/Dynamics/Bullet-C-API.cpp \
./BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp \
./BulletDynamics/Dynamics/btRigidBody.cpp \
./BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp \
./BulletCollision/CollisionShapes/btConvex2dShape.cpp \
./BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp \
./BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp \
./BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp \
./BulletCollision/CollisionShapes/btUniformScalingShape.cpp \
./BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp \
./BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp \
./BulletCollision/CollisionShapes/btConvexInternalShape.cpp \
./BulletCollision/CollisionShapes/btBox2dShape.cpp \
./BulletCollision/CollisionShapes/btTriangleBuffer.cpp \
./BulletCollision/CollisionShapes/btTriangleCallback.cpp \
./BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp \
./BulletCollision/CollisionShapes/btConcaveShape.cpp \
./BulletCollision/CollisionShapes/btShapeHull.cpp \
./BulletCollision/CollisionShapes/btTriangleMeshShape.cpp \
./BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp \
./BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp \
./BulletCollision/CollisionShapes/btConeShape.cpp \
./BulletCollision/CollisionShapes/btCylinderShape.cpp \
./BulletCollision/CollisionShapes/btOptimizedBvh.cpp \
./BulletCollision/CollisionShapes/btTriangleMesh.cpp \
./BulletCollision/CollisionShapes/btCompoundShape.cpp \
./BulletCollision/CollisionShapes/btMultiSphereShape.cpp \
./BulletCollision/CollisionShapes/btCapsuleShape.cpp \
./BulletCollision/CollisionShapes/btTetrahedronShape.cpp \
./BulletCollision/CollisionShapes/btCollisionShape.cpp \
./BulletCollision/CollisionShapes/btStridingMeshInterface.cpp \
./BulletCollision/CollisionShapes/btStaticPlaneShape.cpp \
./BulletCollision/CollisionShapes/btConvexShape.cpp \
./BulletCollision/CollisionShapes/btEmptyShape.cpp \
./BulletCollision/CollisionShapes/btConvexHullShape.cpp \
./BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp \
./BulletCollision/CollisionShapes/btConvexPolyhedron.cpp \
./BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp \
./BulletCollision/CollisionShapes/btSphereShape.cpp \
./BulletCollision/CollisionShapes/btBoxShape.cpp \
./BulletCollision/Gimpact/gim_memory.cpp \
./BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp \
./BulletCollision/Gimpact/btGImpactBvh.cpp \
./BulletCollision/Gimpact/btGenericPoolAllocator.cpp \
./BulletCollision/Gimpact/btContactProcessing.cpp \
./BulletCollision/Gimpact/gim_contact.cpp \
./BulletCollision/Gimpact/gim_box_set.cpp \
./BulletCollision/Gimpact/btTriangleShapeEx.cpp \
./BulletCollision/Gimpact/gim_tri_collision.cpp \
./BulletCollision/Gimpact/btGImpactShape.cpp \
./BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp \
./BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp \
./BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp \
./BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp \
./BulletCollision/BroadphaseCollision/btDispatcher.cpp \
./BulletCollision/BroadphaseCollision/btDbvt.cpp \
./BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp \
./BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp \
./BulletCollision/BroadphaseCollision/btAxisSweep3.cpp \
./BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp \
./BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btManifoldResult.cpp \
./BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp \
./BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp \
./BulletCollision/CollisionDispatch/btGhostObject.cpp \
./BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btCollisionObject.cpp \
./BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp \
./BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp \
./BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp \
./BulletCollision/CollisionDispatch/btCollisionWorld.cpp \
./BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp \
./BulletCollision/CollisionDispatch/btUnionFind.cpp \
./BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp \
./BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp \
./BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp \
./BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp \
./BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp \
./BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp \
./BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp \
./BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp \
./BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp \
./BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp \
./BulletCollision/NarrowPhaseCollision/btConvexCast.cpp \
./BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp \
./BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp \
./BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp \
./MiniCL/MiniCLTask/MiniCLTask.cpp \
./MiniCL/MiniCL.cpp \
./MiniCL/MiniCLTaskScheduler.cpp

all: $(TARGETS)

define PLATFORM_rules
$(1)_SOURCES = $$(SOURCES)
$(1)_OBJS = $$($(1)_SOURCES:%.cpp=./obj/$(1)/%.o)
$(1)_LIB = ../../lib/$(1)/$$(LIB)

$(1): $$($(1)_LIB)

$$($(1)_LIB): $$($(1)_OBJS)
@mkdir -p $$(dir $$@)
$$(TOOL_PREFIX)ar rc $$@ $$^

obj/$(1)/%.o: %.cpp
mkdir -p $$(dir $$@)
$$(TOOL_PREFIX)g++ $$(CFLAGS) -DDAS_PLATFORM_$(1) -MD -o $$@ -c $$<

-include $$($(1)_OBJS:%.o=%.d)
endef

$(foreach platform,$(TARGETS),$(eval $(call PLATFORM_rules,$(platform))))

ifeq ($(shell uname),Linux)
ANDROID_NDK_BASE = $(HOME)/projects/android-ndk-1.6_r1
ANDROID_PLAT=linux-x86
else # windows
ANDROID_NDK_BASE = /cygdrive/c/projects/android-ndk-1.6_r1
ANDROID_PLAT=windows
endif
ANDROID_TOOL_PREFIX = $(ANDROID_NDK_BASE)/build/prebuilt/$(ANDROID_PLAT)/arm-eabi-4.2.1/bin/arm-eabi-
ANDROID_CFLAGS = -march=armv5te -mtune=xscale -msoft-float -fpic -mthumb-interwork \
-ffunction-sections -funwind-tables -fstack-protector -fno-short-enums \
-fno-exceptions -fno-rtti \
-D__ARM_ARCH_5__ -D__ARM_ARCH_5T__ -D__ARM_ARCH_5E__ -D__ARM_ARCH_5TE__ -DANDROID -O2 -DNDEBUG -g \
-I$(ANDROID_NDK_BASE)/build/platforms/android-3/arch-arm/usr/include \
-I../ -I./BroadphaseCollision -I./CollisionDispatch -I./CollisionShapes -I./Gimpact -I./NarrowPhaseCollision

android: TOOL_PREFIX = $(ANDROID_TOOL_PREFIX)
android: CFLAGS += $(ANDROID_CFLAGS)

clean:
rm -fr obj lib

