#include <flow.h>
#include "utest.h"

typedef struct {
	flow_v3   CenterP;
	flow_real Radius;
} sphere;

typedef struct {
	flow_v3 Min;
	flow_v3 Max;
} aabb;

static sphere Make_Sphere(flow_v3 CenterP, flow_real Radius) {
	sphere Result = {CenterP, Radius};
	return Result;
}

static aabb Make_AABB(flow_v3 Min, flow_v3 Max) {
	aabb Result = {Min, Max};
	return Result;
}

static flow_v3 AABB_Get_Closest_Point(aabb AABB, flow_v3 P) {
	flow_v3 Result;
	Result.x = Flow_Min(AABB.Max.x, Flow_Max(AABB.Min.x, P.x));
	Result.y = Flow_Min(AABB.Max.y, Flow_Max(AABB.Min.y, P.y));
	Result.z = Flow_Min(AABB.Max.z, Flow_Max(AABB.Min.z, P.z));
	return Result;
}

static void Test_Closest_Point_To_Triangle(flow_v3 A, flow_v3 B, flow_v3 C, flow_v3 P, flow_v3 ExpectedClosestPoint, flow_u32 ExpectedSet, int* utest_result) {
	flow_v3 P0 = Flow_V3_Sub_V3(A, P);
	flow_v3 P1 = Flow_V3_Sub_V3(B, P);
	flow_v3 P2 = Flow_V3_Sub_V3(C, P);

	flow_u32 ExpectedA = ExpectedSet & 1;
	flow_u32 ExpectedB = (ExpectedSet & 2) >> 1;
	flow_u32 ExpectedC = (ExpectedSet & 4) >> 2;

	flow_u32 Set = 0;
	flow_v3 ClosestPoint = Flow_V3_Add_V3(P, Flow_Closest_Point_From_Triangle_To_Origin_Set(P0, P1, P2, flow_false, &Set));
	ASSERT_EQ(Set, ExpectedSet);
	ASSERT_TRUE(Flow_V3_Is_Close(ClosestPoint, ExpectedClosestPoint, Flow_Sq(2.0e-5f)));

	ClosestPoint = Flow_V3_Add_V3(P, Flow_Closest_Point_From_Triangle_To_Origin_Set(P0, P2, P1, flow_false, &Set));
	ASSERT_EQ(Set, ((ExpectedB << 2) | (ExpectedC << 1) | ExpectedA));
	ASSERT_TRUE(Flow_V3_Is_Close(ClosestPoint, ExpectedClosestPoint, Flow_Sq(2.0e-5f)));

	ClosestPoint = Flow_V3_Add_V3(P, Flow_Closest_Point_From_Triangle_To_Origin_Set(P1, P2, P0, flow_false, &Set));
	ASSERT_EQ(Set, ((ExpectedA << 2) | (ExpectedC << 1) | ExpectedB));
	ASSERT_TRUE(Flow_V3_Is_Close(ClosestPoint, ExpectedClosestPoint, Flow_Sq(2.0e-5f)));

	ClosestPoint = Flow_V3_Add_V3(P, Flow_Closest_Point_From_Triangle_To_Origin_Set(P2, P0, P1, flow_false, &Set));
	ASSERT_EQ(Set, ((ExpectedB << 2) | (ExpectedA << 1) | ExpectedC));
	ASSERT_TRUE(Flow_V3_Is_Close(ClosestPoint, ExpectedClosestPoint, Flow_Sq(2.0e-5f)));

	ClosestPoint = Flow_V3_Add_V3(P, Flow_Closest_Point_From_Triangle_To_Origin_Set(P2, P1, P0, flow_false, &Set));
	ASSERT_EQ(Set, ((ExpectedA << 2) | (ExpectedB << 1) | ExpectedC));
	ASSERT_TRUE(Flow_V3_Is_Close(ClosestPoint, ExpectedClosestPoint, Flow_Sq(2.0e-5f)));
}

UTEST(Flow, TestLongTriangle) {
	flow_v3 A = Flow_V3(100, 1, 0);
	flow_v3 B = Flow_V3(100, 1, 1);
	flow_v3 C = Flow_V3(-100, 1, 0);

	Test_Closest_Point_To_Triangle(A, B, C, Flow_V3(0, 0, 0.1f), Flow_V3(0, 1, 0.1f), 0b0111, utest_result);

	Test_Closest_Point_To_Triangle(A, B, C, Flow_V3(101, 0, 0.5f), Flow_V3(100, 1, 0.5f), 0b0011, utest_result);

	Test_Closest_Point_To_Triangle(A, B, C, Flow_V3(0, 0, -0.1f), Flow_V3(0, 1, 0), 0b0101, utest_result);

	flow_v3 PointBC = Flow_V3(0, 0, 1);
	flow_v3 BC = Flow_V3_Sub_V3(C, B);
	flow_v3 ClosestBC = Flow_V3_Add_V3(B, Flow_V3_Mul_S(BC, Flow_V3_Dot(Flow_V3_Sub_V3(PointBC, B), BC) / Flow_V3_Sq_Mag(BC)));

	Test_Closest_Point_To_Triangle(A, B, C, PointBC, ClosestBC, 0b0110, utest_result);

	Test_Closest_Point_To_Triangle(A, B, C, Flow_V3(101, 0, -1), A, 0b0001, utest_result);

	Test_Closest_Point_To_Triangle(A, B, C, Flow_V3(101, 0, 2), B, 0b0010, utest_result);

	Test_Closest_Point_To_Triangle(A, B, C, Flow_V3(-101, 0, 0), C, 0b0100, utest_result);
}

UTEST(Flow, TestNearColinearTriangle) {
	// A very long triangle that is nearly colinear
	flow_v3 a = Flow_V3(99.9999847f, 0.946687222f, 99.9999847f);
	flow_v3 b = Flow_V3(-100.010002f, 0.977360725f, -100.010002f);
	flow_v3 c = Flow_V3(-100.000137f, 0.977310658f, -100.000137f);

	flow_v3 ac = Flow_V3_Sub_V3(c, a);
	flow_v3 ExpectedClosest = Flow_V3_Add_V3(a, Flow_V3_Mul_S(ac, Flow_V3_Dot(Flow_V3_Negate(a), ac) / Flow_V3_Sq_Mag(ac)));

	Test_Closest_Point_To_Triangle(a, b, c, Flow_V3_Zero(), ExpectedClosest, 0b0101, utest_result);
}

UTEST(Flow, TestSmallTriangleWithPlaneGoingThroughOrigin) {
	// A small but non-degenerate triangle whose plane almost goes through the origin
	flow_v3 a = Flow_V3(-0.132395342f, -0.294095188f, -0.164812326f);
	flow_v3 b = Flow_V3(-0.126054004f, -0.283950001f, -0.159065604f);
	flow_v3 c = Flow_V3(-0.154956535f, -0.284792334f, -0.160523415f);

	flow_real u, v, w;
	Flow_Get_Triangle_Barycentric_From_Origin(a, b, c, &u, &v, &w);

	flow_v3 p = Flow_V3_Add_V3(Flow_V3_Add_V3(Flow_V3_Mul_S(a, u), Flow_V3_Mul_S(b, v)), Flow_V3_Mul_S(c, w));
	ASSERT_TRUE(Flow_V3_Is_Close(p, Flow_V3_Zero(), Flow_Sq(flow_constant(1e-6))));

	// Closest point should be outside triangle
	ASSERT_TRUE((u < 0.0f || v > 0.0f || w < 0.0f));
}

UTEST(Flow, TestGJKIntersectsSphere) {
	flow_allocator Allocator = { Flow_Malloc, Flow_Free };

	flow_arena Arena = { 0 };
	Flow_Arena_Init(&Arena, Allocator);

	flow_gjk_support S1 = Flow_GJK_Sphere(&Arena, Flow_V3_Zero(), 1.0f);
	
	flow_v3 C2 = Flow_V3(10.0f, 10.0f, 10.0f);
	flow_gjk_support S2 = Flow_GJK_Sphere(&Arena, C2, 1.0f);

	flow_real l = 2.0f / Flow_Sqrt(3.0f);
	flow_v3 C3 = Flow_V3(l, l, l);
	flow_gjk_support S3 = Flow_GJK_Sphere(&Arena, C3, 1.0f);

	{
		flow_v3 v = Flow_V3_Zero();
		ASSERT_FALSE(Flow_GJK_Intersects(&S1, &S2, 1.0e-4f, &v));
	}

	{
		flow_v3 v = Flow_V3_Zero();
		ASSERT_TRUE(Flow_GJK_Intersects(&S1, &S3, 1.0e-4f, &v));
	}

	{
		flow_v3 v = Flow_V3_Zero();
		flow_closest_points ClosestPoints = Flow_GJK_Get_Closest_Points(&S1, &S2, 1.0e-4f, FLOW_MAX_REAL, &v, NULL);
		ASSERT_TRUE(Flow_Is_Close(Flow_V3_Mag(C2) - 2.0f, Flow_Sqrt(ClosestPoints.DistSq), 1.0e-4f));
		ASSERT_TRUE(Flow_V3_Is_Close(Flow_V3_Norm(C2), ClosestPoints.P1, 1.0e-4f));
		ASSERT_TRUE(Flow_V3_Is_Close(Flow_V3_Sub_V3(C2, Flow_V3_Norm(C2)), ClosestPoints.P2, 1.0e-4f));
	}

	{
		flow_v3 v = Flow_V3_Zero();
		flow_closest_points ClosestPoints = Flow_GJK_Get_Closest_Points(&S1, &S3, 1.0e-4f, FLOW_MAX_REAL, &v, NULL);
		ASSERT_TRUE(Flow_Is_Close(0.0f, Flow_Sqrt(ClosestPoints.DistSq), 1.0e-4f));
		ASSERT_TRUE(Flow_V3_Is_Close(Flow_V3_Norm(C2), ClosestPoints.P1, 1.0e-4f));
		ASSERT_TRUE(Flow_V3_Is_Close(Flow_V3_Norm(C2), ClosestPoints.P2, 1.0e-4f));
	}

	Flow_Arena_Release(&Arena);
}

static flow_bool Collide_Box_Sphere(flow_m4* Transform, aabb AABB, sphere Sphere) {
	flow_allocator Allocator = { Flow_Malloc, Flow_Free };

	flow_arena Arena = { 0 };
	Flow_Arena_Init(&Arena, Allocator);

	flow_gjk_support SupportA = Flow_GJK_AABB(&Arena, AABB.Min, AABB.Max);
	flow_gjk_support SupportB = Flow_GJK_Sphere(&Arena, Sphere.CenterP, Sphere.Radius);

	flow_m3 TransformM = Flow_M3_From_M4(Transform);

	flow_gjk_support TransformedSupportA = Flow_GJK_Transform(&Arena, SupportA, Transform);
	flow_gjk_support TransformedSupportB = Flow_GJK_Transform(&Arena, SupportB, Transform);

	//EPA test
	flow_penetration_test PenetrationTest = Flow_EPA_Penetration_Test(&TransformedSupportA, &TransformedSupportB, 1.0e-2f, FLOW_EPSILON);

	//Analytical solution
	flow_v3 P2 = AABB_Get_Closest_Point(AABB, Sphere.CenterP);
	flow_v3 V = Flow_V3_Sub_V3(Sphere.CenterP, P2);
	flow_bool Intersected = Flow_V3_Sq_Mag(V) <= Flow_Sq(Sphere.Radius);

	Flow_Assert(Intersected == PenetrationTest.Intersected);
	if (Intersected && PenetrationTest.Intersected) {
		flow_v3 P1 = Flow_V3_Sub_V3(Sphere.CenterP, Flow_V3_Mul_S(Flow_V3_Norm(V), Sphere.Radius));

		//Transform analytical solution
		V = Flow_V3_Mul_M3(V, &TransformM);
		P1 = Flow_V4_Mul_M4(Flow_V4_From_V3(P1, 1.0f), Transform).xyz;
		P2 = Flow_V4_Mul_M4(Flow_V4_From_V3(P2, 1.0f), Transform).xyz;

		// Check angle between v1 and v2
		flow_real Angle = Flow_V3_Angle_Between(PenetrationTest.V, V);
		Flow_Assert(Flow_To_Degrees(Angle) < 0.1f);

		// Check delta between contact on A
		flow_v3 DeltaP1 = Flow_V3_Sub_V3(P1, PenetrationTest.P1);
		Flow_Assert(Flow_V3_Mag(DeltaP1) < 8.0e-4f);

		// Check delta between contact on B
		flow_v3 DeltaP2 = Flow_V3_Sub_V3(P2, PenetrationTest.P2);
		Flow_Assert(Flow_V3_Mag(DeltaP2) < 8.0e-4f);
	}

	Flow_Arena_Release(&Arena);
	return PenetrationTest.Intersected;
}

static void Collide_Boxes_With_Spheres(flow_m4* Transform, int* utest_result) {

	{
		// Sphere just missing face of box
		aabb Box = Make_AABB(Flow_V3(-2, -3, -4), Flow_V3(2, 3, 4));
		sphere Sphere = Make_Sphere(Flow_V3(4, 0, 0), 1.99f);
		ASSERT_FALSE(Collide_Box_Sphere(Transform, Box, Sphere));
	}

	{
		// Sphere just touching face of box
		aabb Box = Make_AABB(Flow_V3(-2, -3, -4), Flow_V3(2, 3, 4));
		sphere Sphere = Make_Sphere(Flow_V3(4, 0, 0), 2.01f);
		ASSERT_TRUE(Collide_Box_Sphere(Transform, Box, Sphere));
	}

	{
		// Sphere deeply penetrating box on face
		aabb Box = Make_AABB(Flow_V3(-2, -3, -4), Flow_V3(2, 3, 4));
		sphere Sphere = Make_Sphere(Flow_V3(3, 0, 0), 2);
		ASSERT_TRUE(Collide_Box_Sphere(Transform, Box, Sphere));
	}

	{
		// Sphere just missing box on edge
		aabb Box = Make_AABB(Flow_V3(1, 1, -2), Flow_V3(2, 2, 2));
		sphere Sphere = Make_Sphere(Flow_V3(4, 4, 0), Flow_Sqrt(8.0f) - 0.01f);
		ASSERT_FALSE(Collide_Box_Sphere(Transform, Box, Sphere));
	}

	{
		// Sphere just penetrating box on edge
		aabb Box = Make_AABB(Flow_V3(1, 1, -2), Flow_V3(2, 2, 2));
		sphere Sphere = Make_Sphere(Flow_V3(4, 4, 0), Flow_Sqrt(8.0f) + 0.01f);
		ASSERT_TRUE(Collide_Box_Sphere(Transform, Box, Sphere));
	}

	{
		// Sphere just missing box on vertex
		aabb Box = Make_AABB(Flow_V3(1, 1, 1), Flow_V3(2, 2, 2));
		sphere Sphere = Make_Sphere(Flow_V3(4, 4, 4), Flow_Sqrt(12.0f) - 0.01f);
		ASSERT_FALSE(Collide_Box_Sphere(Transform, Box, Sphere));
	}

	{
		// Sphere just penetrating box on vertex
		aabb Box = Make_AABB(Flow_V3(1, 1, 1), Flow_V3(2, 2, 2));
		sphere Sphere = Make_Sphere(Flow_V3(4, 4, 4), Flow_Sqrt(12.0f) + 0.01f);
		ASSERT_TRUE(Collide_Box_Sphere(Transform, Box, Sphere));
	}
}

UTEST(Flow, TestEPASphereBox) {
	flow_m4 Identity = {
		1, 0, 0, 0, 
		0, 1, 0, 0, 
		0, 0, 1, 0, 
		0, 0, 0, 1
	};
	Collide_Boxes_With_Spheres(&Identity, utest_result);

	flow_m4 Transform = {
		flow_constant(0.843391), flow_constant(0.268650), flow_constant(-0.465315), 0,
		flow_constant(0.000000), flow_constant(0.866025), flow_constant(0.500000), 0,
		flow_constant(0.537300), flow_constant(-0.421696), flow_constant(0.730398), 0,
		flow_constant(1.361121), flow_constant(0.180594), flow_constant(0.000000), 1
	};

	flow_m4 Transform2 = {
		flow_constant(0.843391), flow_constant(0.268650), flow_constant(-0.465315), 0,
		flow_constant(0.000000), flow_constant(0.866025), flow_constant(0.500000),  0, 
		flow_constant(0.537300), flow_constant(-0.421696), flow_constant(0.730398), 0, 
		flow_constant(-0.780626), flow_constant(0.464092), flow_constant(0.000000), 1
	};

	Collide_Boxes_With_Spheres(&Transform, utest_result);
	Collide_Boxes_With_Spheres(&Transform2, utest_result);
}



#ifndef __cplusplus
UTEST_MAIN();
#endif