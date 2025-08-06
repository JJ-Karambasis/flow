#ifndef FLOW_H
#define FLOW_H

#if defined(__clang__)
#define FLOW_COMPILER_CLANG
#elif defined(_MSC_VER)
#define FLOW_COMPILER_MSVC
#endif

#ifdef FLOW_COMPILER_MSVC
#pragma warning(push)
#pragma warning(disable : 4201 4820 4587)
#endif

#ifdef FLOW_STATIC_DEF
#define FLOWDEF static
#else
#ifdef __cplusplus
#define FLOWDEF extern "C"
#else
#define FLOWDEF extern
#endif
#endif

/* Generate unique names */
#define flow_static_assert_line(condition, line) \
typedef char static_assert_##line[(condition) ? 1 : -1]

#define flow_static_assert(condition) flow_static_assert_line(condition, __LINE__)

#if defined(FLOW_COMPILER_MSVC)
# if defined(__SANITIZE_ADDRESS__)
#  define FLOW_ASAN_ENABLED
#  define FLOW_NO_ASAN __declspec(no_sanitize_address)
# else
#  define FLOW_NO_ASAN
# endif
#elif defined(FLOW_COMPILER_CLANG)
# if defined(__has_feature)
#  if __has_feature(address_sanitizer) || defined(__SANITIZE_ADDRESS__)
#   define FLOW_ASAN_ENABLED
#  endif
# endif
# define FLOW_NO_ASAN __attribute__((no_sanitize("address")))
#else
# define FLOW_NO_ASAN
#endif

#ifdef FLOW_ASAN_ENABLED
#ifndef Flow_Asan_Poison_Memory_Region
#pragma comment(lib, "clang_rt.asan_dynamic-x86_64.lib")
void __asan_poison_memory_region(void const volatile *addr, size_t size);
void __asan_unpoison_memory_region(void const volatile *addr, size_t size);
#define Flow_Asan_Poison_Memory_Region(addr, size)   __asan_poison_memory_region((addr), (size))
#define Flow_Asan_Unpoison_Memory_Region(addr, size) __asan_unpoison_memory_region((addr), (size))
#endif
#else
#define Flow_Asan_Poison_Memory_Region(addr, size)   ((void)(addr), (void)(size))
#define Flow_Asan_Unpoison_Memory_Region(addr, size) ((void)(addr), (void)(size))
#endif

#include <stdint.h>

typedef int8_t  flow_s8;
typedef int16_t flow_s16;
typedef int32_t flow_s32;
typedef int64_t flow_s64;

typedef uint8_t  flow_u8;
typedef uint16_t flow_u16;
typedef uint32_t flow_u32;
typedef uint64_t flow_u64;

typedef float flow_f32;
typedef double flow_f64;

typedef flow_s8 flow_bool8;
typedef flow_s32 flow_bool;

#define flow_false 0
#define flow_true 1

#ifdef FLOW_USE_F64
typedef flow_f64 flow_real;
#else
typedef flow_f32 flow_real;
#endif

flow_static_assert(sizeof(flow_s8)  == 1);
flow_static_assert(sizeof(flow_s16) == 2);
flow_static_assert(sizeof(flow_s32) == 4);
flow_static_assert(sizeof(flow_s64) == 8);

flow_static_assert(sizeof(flow_u8)  == 1);
flow_static_assert(sizeof(flow_u16) == 2);
flow_static_assert(sizeof(flow_u32) == 4);
flow_static_assert(sizeof(flow_u64) == 8);

flow_static_assert(sizeof(flow_f32) == 4);
flow_static_assert(sizeof(flow_f64) == 8);

typedef struct flow_v3 flow_v3;

struct flow_v3 {
	union {
		flow_real Data[3];
		struct {
			flow_real x, y, z;
		};
	};

#ifdef __cplusplus
	flow_v3() : x(0), y(0), z(0) {}
	flow_v3(flow_real X, flow_real Y, flow_real Z) : x(X), y(Y), z(Z) {}
#ifdef FLOW_VEC3_CLASS_EXTRA
	FLOW_VEC3_CLASS_EXTRA
#endif
#endif
};

typedef struct flow_quat flow_quat;
struct flow_quat {
	union {
		flow_real Data[4];
		struct { flow_v3 v; flow_real s; };
		struct { flow_real x, y, z, w; };
	};

#ifdef __cplusplus
	flow_quat() : x(0), y(0), z(0), w(0) {}
	flow_quat(flow_real X, flow_real Y, flow_real Z, flow_real W) : x(X), y(Y), z(Z), w(W) {}
#ifdef FLOW_QUAT_CLASS_EXTRA
	FLOW_QUAT_CLASS_EXTRA
#endif
#endif
};

typedef void* flow_allocate_memory_func(size_t Size, void* UserData);
typedef void  flow_free_memory_func(void* Memory, void* UserData);
typedef struct {
	flow_allocate_memory_func* AllocateMemory;
	flow_free_memory_func* 	   FreeMemory;
	void* 				  	   UserData;
} flow_allocator;

typedef struct flow_system flow_system;
typedef struct {
	flow_allocator* Allocator;	
} flow_system_create_info;

FLOWDEF flow_system* Flow_System_Create(const flow_system_create_info* CreateInfo);
FLOWDEF void Flow_System_Delete(flow_system* System);
FLOWDEF void Flow_System_Update(flow_system* System, flow_f64 dt);
FLOWDEF void Flow_System_Set_Gravity(flow_system* System, flow_v3 Gravity);
FLOWDEF void Flow_System_Get_Gravity(flow_system* System, flow_v3* Gravity);

typedef enum {
	FLOW_COLLIDER_TYPE_SPHERE,
	FLOW_COLLIDER_TYPE_BOX,
	FLOW_COLLIDER_TYPE_COUNT
} flow_collider_type;

typedef struct {
	flow_real Radius;
	flow_real Density;
} flow_sphere;

typedef struct {
	flow_v3   HalfExtent;
	flow_real Density;
} flow_box;

typedef struct flow_transformed_collider flow_transformed_collider;
typedef struct {
	flow_u32 				   Count;
	flow_transformed_collider* Colliders;
} flow_compound;

typedef struct {
	flow_collider_type Type;
	union {
		flow_sphere   Sphere;
		flow_box      Box;
		flow_compound Compound;
	};
} flow_collider;

struct flow_transformed_collider {
	flow_v3       Position;
	flow_quat     Orientation;
	flow_collider Collider;
};

typedef enum {
	FLOW_BODY_STATIC,
	FLOW_BODY_DYNAMIC
} flow_body_type;

typedef struct {
	flow_v3        Position;
	flow_quat      Orientation;
	flow_v3        Scale;
	flow_body_type Type;
	flow_collider  Collider;
	flow_real 	   Friction;
	void* 	       UserData;
} flow_body_create_info;

typedef struct {
	flow_v3   	  Position;
	flow_quat 	  Orientation;
	flow_v3   	  Scale;

	flow_body_type Type;

	flow_v3 LinearVelocity;
	flow_v3 AngularVelocity;

	flow_v3 Force;
	flow_v3 Torque;

	flow_real Friction;

	flow_collider Collider;
	void* 	      UserData;
} flow_body;

typedef flow_u64 flow_body_id;

FLOWDEF flow_body_id Flow_Create_Body(flow_system* System, const flow_body_create_info* CreateInfo);
FLOWDEF flow_body*   Flow_Get_Body(flow_system* System, flow_body_id ID);
FLOWDEF void 		 Flow_Delete_Body(flow_system* System, flow_body_id ID);

typedef struct flow_convex_hull flow_convex_hull;
typedef struct flow_triangle_mesh flow_triangle_mesh;

#ifdef FLOW_DEBUG_RENDERING

//Debug rendering api

typedef enum {
	FLOW_DEBUG_CMD_TYPE_COUNT,
	FLOW_DEBUG_CMD_PENETRATION
} flow_debug_cmd_type;

typedef struct {
	flow_debug_cmd_type Type;
	union {
		struct {
			flow_body_id BodyA;
			flow_body_id BodyB;
			flow_v3 	 P1;
			flow_v3 	 P2;
			flow_v3 	 V;
		} Penetration;
	};
} flow_debug_cmd;

typedef struct {
	size_t 			CmdCapacity;
	size_t 			CmdCount;
	flow_debug_cmd* Cmds;
} flow_debug_renderer;

FLOWDEF flow_debug_renderer* Flow_Get_Renderer(flow_system* System);

#endif

#ifdef FLOW_COMPILER_MSVC
#pragma warning(pop)
#endif

#endif

#ifdef FLOW_IMPLEMENTATION

#ifdef FLOW_USE_F64
#define FLOW_EPSILON 2.2204460492503131e-16
#define FLOW_MAX_REAL 1.7976931348623157e+308
#else
#define FLOW_EPSILON 1.19209290e-07F
#define FLOW_MAX_REAL 3.40282346638528860e+38F
#endif 

#define flow_constant(x) ((flow_real)(x))

#ifdef FLOW_COMPILER_MSVC
#pragma warning(push)
#pragma warning(disable : 4201 4820 4255 4456 4100 5246 4365 4505 5045 4582 4061)
#endif

#ifdef FLOW_COMPILER_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-braces"
#pragma clang diagnostic ignored "-Wunused-static"
#endif

#ifndef FLOW_MAX_VELOCITY_ITERATIONS
#define FLOW_MAX_VELOCITY_ITERATIONS 6
#endif

#ifndef FLOW_MAX_POSITION_ITERATIONS
#define FLOW_MAX_POSITION_ITERATIONS 2
#endif

#define FLOW_PENETRATION_SLOP flow_constant(0.02)
#define FLOW_BAUMGUARTE flow_constant(0.2)
#define FLOW_MANIFOLD_TOLERANCE_SQ flow_constant(1.0e-6)
#define FLOW_COLLISION_TOLERANCE flow_constant(1.0e-4)
#define FLOW_PENETRATION_TOLERANCE flow_constant(1.0e-4)

#define FLOW_DEFAULT_ALIGNMENT 16

#define FLOW_PI flow_constant(3.14159265359)

#define Flow_Radians_Const flow_constant(0.0174533)
#define Flow_To_Radians(degrees) ((degrees)*Flow_Radians_Const)

#define Flow_Degrees_Const flow_constant(57.295779513)
#define Flow_To_Degrees(radians) ((radians)*Flow_Degrees_Const)

#define Flow_Max(a, b) (((a) > (b)) ? (a) : (b))
#define Flow_Min(a, b) (((a) < (b)) ? (a) : (b))
#define Flow_Clamp(min, v, max) Flow_Min(Flow_Max(min, v), max)
#define Flow_Saturate(v) Flow_Clamp(0, v, 1)

#define Flow_Sq(x) ((x)*(x))

#define Flow_Is_Pow2(x) (((x) != 0) && (((x) & ((x) - 1)) == 0))
#define Flow_Align_Pow2(x, a) (((x) + (a)-1) & ~((a)-1))

#define Flow_Abs(v) (((v) < 0) ? -(v) : (v))

#define Flow_Array_Count(array) (sizeof(array)/sizeof((array)[0]))

#ifndef Flow_Assert
#include <assert.h>
#define Flow_Assert(c) assert(c)
#endif

#define Flow_Invalid_Default_Case default: { Flow_Assert(flow_false); } break
#define Flow_Not_Implemented Flow_Assert(!"Not Implemented")

#ifndef Flow_Memset
#include <string.h>
#define Flow_Memset(dst, value, size) memset(dst, value, size)
#endif

#ifndef Flow_Memcpy
#include <string.h>
#define Flow_Memcpy(dst, src, size) memcpy(dst, src, size)
#endif

#ifndef Flow_Sqrt
#include <math.h>
#ifdef FLOW_USE_F64
#define Flow_Sqrt(x) sqrt(x)
#else
#define Flow_Sqrt(x) sqrtf(x)
#endif
#endif

#ifndef Flow_Cos
#include <math.h>
#ifdef FLOW_USE_F64
#define Flow_Cos(x) cos(x)
#else
#define Flow_Cos(x) cosf(x)
#endif
#endif

#ifndef Flow_Sin
#include <math.h>
#ifdef FLOW_USE_F64
#define Flow_Sin(x) sin(x)
#else
#define Flow_Sin(x) sinf(x)
#endif
#endif

#ifndef Flow_ACos
#include <math.h>
#ifdef FLOW_USE_F64
#define Flow_ACos(x) acos(x)
#else
#define Flow_ACos(x) acosf(x)
#endif
#endif

#ifndef Flow_Is_Nan
#include <math.h>
#define Flow_Is_Nan(x) isnan(x)
#endif

#define Flow_SLL_Pop_Front_N(First, Next) ((First) = (First)->Next)
#define Flow_SLL_Push_Front_N(First, Node, Next) (Node->Next = First, First = Node)

#define Flow_SLL_Push_Back_NP(First, Last, Node, Next, Prev) (!First ? (First = Last = Node) : (Last->Next = Node, Last = Node))

#define Flow_SLL_Push_Back(First, Last, Node) Flow_SLL_Push_Back_NP(First, Last, Node, Next, Prev)
#define Flow_SLL_Push_Front(First, Node) Flow_SLL_Push_Front_N(First, Node, Next)
#define Flow_SLL_Pop_Front(First) Flow_SLL_Pop_Front_N(First, Next)

#define Flow_Combine_Friction(f1, f2) Flow_Sqrt((f1)*(f2))

#ifndef FLOW_NO_STBLIB
#include <stdlib.h>

static void* Flow_Malloc(size_t Size, void* UserData) {
	void* Result = malloc(Size);
	return Result;
}

static void Flow_Free(void* Memory, void* UserData) {
	free(Memory);
}
#endif

static flow_bool Flow_Equal_Zero_Approx(flow_real Value, flow_real Epsilon) {
    return Flow_Abs(Value) <= Epsilon;
}

static flow_bool Flow_Equal_Zero_Eps(flow_real Value) {
    return Flow_Equal_Zero_Approx(Value, FLOW_EPSILON);
}

static flow_bool Flow_Equal_Zero_Eps_Sq(flow_real SqValue) {
    return Flow_Equal_Zero_Approx(SqValue, Flow_Sq(FLOW_EPSILON));
}

static size_t Flow_Align(size_t Value, size_t Alignment) {
    size_t Remainder = Value % Alignment;
    return Remainder ? Value + (Alignment-Remainder) : Value;
}

static flow_u64 Flow_Hash_U64(flow_u64 x) {
	x ^= x >> 30;
	x *= 0xbf58476d1ce4e5b9ULL;
	x ^= x >> 27;
	x *= 0x94d049bb133111ebULL;
	x ^= x >> 31;
	return x;
}

static flow_u32 Flow_Ceil_Pow2_U32(flow_u32 V) {
    V--;
    V |= V >> 1;
    V |= V >> 2;
    V |= V >> 4;
    V |= V >> 8;
    V |= V >> 16;
    V++;
    return V;
}

static flow_bool Flow_Is_Close(flow_real A, flow_real B, flow_real ToleranceSq) {
	flow_bool Result = Flow_Abs(B-A) <= ToleranceSq;
	return Result;
}

static void* Flow_Allocate_Memory(flow_allocator* Allocator, size_t Size) {
	void* Result = Allocator->AllocateMemory(Size, Allocator->UserData);
	return Result;
}

static void Flow_Free_Memory(flow_allocator* Allocator, void* Memory) {
	Allocator->FreeMemory(Memory, Allocator->UserData);
}

typedef struct flow_arena_block flow_arena_block;
struct flow_arena_block {
	flow_u8* Memory;
	size_t   Used;
	size_t   Size;
	flow_arena_block* Next;
};

typedef struct {
	flow_allocator 	  Allocator;
	flow_arena_block* FirstBlock;
	flow_arena_block* LastBlock;
	flow_arena_block* CurrentBlock;
} flow_arena;

static void Flow_Arena_Init(flow_arena* Arena, flow_allocator Allocator) {
	Flow_Memset(Arena, 0, sizeof(flow_arena));
	Arena->Allocator = Allocator;
}

static void Flow_Arena_Release(flow_arena* Arena) {
	flow_allocator* Allocator = &Arena->Allocator;

	flow_arena_block* Block = Arena->FirstBlock;
	while (Block) {
		flow_arena_block* BlockToDelete = Block;
		Block = Block->Next;

		Flow_Free_Memory(Allocator, BlockToDelete);
	}
	Flow_Memset(Arena, 0, sizeof(flow_arena));
}

static flow_arena_block* Flow_Arena_Get_Current_Block(flow_arena* Arena, size_t Size, size_t Alignment) {
	flow_arena_block* Block = Arena->CurrentBlock;
	while (Block) {
		size_t Used = Flow_Align_Pow2(Block->Used, Alignment);
		if ((Used + Size) <= Block->Size) {
			return Block;
		}
		Block = Block->Next;
	}
	return Block;
}

static flow_arena_block* Flow_Arena_Create_New_Block(flow_arena* Arena, size_t BlockSize) {
	flow_allocator* Allocator = &Arena->Allocator;

	size_t AllocationSize = BlockSize + sizeof(flow_arena_block);
	flow_arena_block* Block = (flow_arena_block*)Flow_Allocate_Memory(Allocator, AllocationSize);
	if (!Block) return NULL;

	Block->Memory = (flow_u8*)(Block + 1);
	Block->Used = 0;
	Block->Size = BlockSize;

	Flow_SLL_Push_Back(Arena->FirstBlock, Arena->LastBlock, Block);
	Flow_Asan_Poison_Memory_Region(Block, AllocationSize);

	return Block;
}

static void* Flow_Arena_Push_Aligned(flow_arena* Arena, size_t Size, size_t Alignment) {
	Flow_Assert(Flow_Is_Pow2(Alignment));
	
	flow_arena_block* CurrentBlock = Flow_Arena_Get_Current_Block(Arena, Size, Alignment);
	if (!CurrentBlock) {
		size_t BlockSize = Flow_Align(Size+Alignment, 1024*1024);
		CurrentBlock = Flow_Arena_Create_New_Block(Arena, BlockSize);
		if (!CurrentBlock) {
			//todo: Diagnostic and error logging
			return NULL;
		}
	}

	Arena->CurrentBlock = CurrentBlock;
	CurrentBlock->Used = Flow_Align_Pow2(CurrentBlock->Used, Alignment);
	Flow_Assert(CurrentBlock->Used + Size <= CurrentBlock->Size);

	void* Result = CurrentBlock->Memory + CurrentBlock->Used;
	CurrentBlock->Used += Size;
	Flow_Asan_Unpoison_Memory_Region(Result, Size);

	//Clear to zero
	Flow_Memset(Result, 0, Size);

	return Result;
}

static void* Flow_Arena_Push(flow_arena* Arena, size_t Size) {
	return Flow_Arena_Push_Aligned(Arena, Size, FLOW_DEFAULT_ALIGNMENT);
}

static void Flow_Arena_Clear(flow_arena* Arena) {
	for (flow_arena_block* Block = Arena->FirstBlock; Block; Block = Block->Next) {
		Block->Used = 0;
		Flow_Asan_Unpoison_Memory_Region(Block->Memory, Block->Size);
	}
	Arena->CurrentBlock = Arena->FirstBlock;
}

#define Flow_Arena_Push_Struct(arena, type) (type*)Flow_Arena_Push(arena, sizeof(type))
#define Flow_Arena_Push_Array(arena, count, type) (type*)Flow_Arena_Push(arena, sizeof(type)*(count))

typedef struct {
	union {
		flow_u64 ID;
		struct {
			flow_u32 Index;
			flow_u32 Generation;
		};
	};
} flow_pool_id;

#define FLOW_INVALID_POOL_INDEX ((flow_u32)-1)
typedef struct {
	flow_allocator Allocator;
	flow_pool_id*  IDs;
	flow_body*     Bodies;
	flow_u32 Count;
	flow_u32 MaxUsed;
	flow_u32 Capacity;
	flow_u32 FirstFreeIndex;
} flow_body_pool;

static void Flow_Body_Pool_Init(flow_body_pool* Pool, flow_allocator Allocator) {
	Flow_Memset(Pool, 0, sizeof(flow_body_pool));
	Pool->Allocator = Allocator;
	Pool->FirstFreeIndex = FLOW_INVALID_POOL_INDEX;
}

static flow_body_id Flow_Body_Pool_Allocate(flow_body_pool* Pool) {
	flow_u32 Index;
	if (Pool->FirstFreeIndex != FLOW_INVALID_POOL_INDEX) {
		Index = Pool->FirstFreeIndex;
		Pool->FirstFreeIndex = Pool->IDs[Index].Index;
	} else {
		if (Pool->MaxUsed == Pool->Capacity) {
			flow_u32 NewCapacity = Flow_Max(Pool->Capacity*2, 32);
		
			size_t AllocationSize = (sizeof(flow_pool_id) + sizeof(flow_body))*NewCapacity;
			flow_pool_id* NewIDs = (flow_pool_id *)Flow_Allocate_Memory(&Pool->Allocator, AllocationSize);
			flow_body* NewBodies = (flow_body*)(NewIDs + NewCapacity);

			for (flow_u32 i = 0; i < NewCapacity; i++) {
				NewIDs[i].Generation = 1;
				NewIDs[i].Index = FLOW_INVALID_POOL_INDEX;
			}

			if (Pool->IDs) {
				Flow_Memcpy(NewIDs, Pool->IDs, Pool->Capacity * sizeof(flow_pool_id));
				Flow_Memcpy(NewBodies, Pool->Bodies, Pool->Capacity * sizeof(flow_body));
				Flow_Free_Memory(&Pool->Allocator, Pool->IDs);
			}

			Pool->IDs = NewIDs;
			Pool->Bodies = NewBodies;

			Pool->Capacity = NewCapacity;
		}

		Index = Pool->MaxUsed++;
	}

	flow_pool_id* ID = Pool->IDs + Index;
	ID->Index = Index;

	flow_body* Body = Pool->Bodies + Index;
	Flow_Memset(Body, 0, sizeof(flow_body));

	Pool->Count++;

	return ID->ID;
}

static void Flow_Body_Pool_Free(flow_body_pool* Pool, flow_body_id BodyID) {
	if (!BodyID) return;

	flow_pool_id ID = { BodyID };
	flow_pool_id* PoolID = Pool->IDs + ID.Index;
	if (PoolID->ID == BodyID) {
		PoolID->Generation++;
		if (PoolID->Generation == 0) PoolID->Generation = 1;
		PoolID->Index = Pool->FirstFreeIndex;
		Pool->FirstFreeIndex = ID.Index;
		
		Pool->Count--;
	}
}

static flow_body* Flow_Body_Pool_Get(flow_body_pool* Pool, flow_body_id BodyID) {
	if (!BodyID) return NULL;

	flow_pool_id ID = { BodyID };
	flow_pool_id* PoolID = Pool->IDs + ID.Index;
	if (PoolID->ID == BodyID) {
		return Pool->Bodies + ID.Index;
	}
	return NULL;
}

static flow_body_id Flow_Body_Pool_Get_ID(flow_body_pool* Pool, flow_body* Body) {
	size_t Index = (size_t)(Body - Pool->Bodies);
	return Pool->IDs[Index].ID;
}

static flow_v3 Flow_V3(flow_real x, flow_real y, flow_real z) {
	flow_v3 Result = { x, y, z };
	return Result;
}

static flow_v3 Flow_V3_Zero() {
	flow_v3 Result = { 0, 0, 0 };
	return Result;
}

static flow_v3 Flow_V3_Add_V3(flow_v3 a, flow_v3 b) {
	flow_v3 Result = { a.x + b.x, a.y + b.y, a.z + b.z };
	return Result;
}

static flow_v3 Flow_V3_Sub_V3(flow_v3 a, flow_v3 b) {
	flow_v3 Result = { a.x - b.x, a.y - b.y, a.z - b.z };
	return Result;
}

static flow_v3 Flow_V3_Mul_S(flow_v3 v, flow_real s) {
	flow_v3 Result = { v.x * s, v.y * s, v.z * s };
	return Result;
}

static flow_v3 Flow_V3_Mul_V3(flow_v3 a, flow_v3 b) {
	flow_v3 Result = { a.x * b.x, a.y * b.y, a.z * b.z };
	return Result;
}

static flow_real Flow_V3_Dot(flow_v3 a, flow_v3 b) {
	flow_real Result = { a.x * b.x + a.y * b.y + a.z * b.z };
	return Result;
}

static flow_real Flow_V3_Sq_Mag(flow_v3 v) {
	flow_real Result = Flow_V3_Dot(v, v);
	return Result;
}

static flow_real Flow_V3_Mag(flow_v3 v) {
	flow_real Result = Flow_Sqrt(Flow_V3_Sq_Mag(v));
	return Result;
}

static flow_v3 Flow_V3_Norm(flow_v3 v) {
	flow_real SqLength = Flow_V3_Sq_Mag(v);
	if (Flow_Equal_Zero_Eps_Sq(SqLength)) {
		return Flow_V3_Zero();
	}

	flow_real InvLength = flow_constant(1.0) / Flow_Sqrt(SqLength);
	return Flow_V3_Mul_S(v, InvLength);
}

static flow_v3 Flow_V3_Negate(flow_v3 v) {
	flow_v3 Result = { -v.x, -v.y, -v.z };
	return Result;
}

static flow_v3 Flow_V3_Cross(flow_v3 A, flow_v3 B) {
	flow_v3 Result = { A.y*B.z - A.z*B.y, A.z*B.x - A.x*B.z, A.x*B.y - A.y*B.x };
	return Result;
}

static flow_v3 Flow_V3_Get_Perp(flow_v3 Direction) {
	flow_v3  Z = Flow_V3_Norm(Direction);
	flow_v3  Up = Flow_V3(0, 1, 0);

	flow_real Diff = flow_constant(1) - Flow_Abs(Flow_V3_Dot(Z, Up));
	if(Flow_Equal_Zero_Eps(Diff))
		Up = Flow_V3(0, 0, 1);

	flow_v3 Result = Flow_V3_Negate(Flow_V3_Norm(Flow_V3_Cross(Z, Up)));
	return Result;
}

static flow_bool Flow_V3_Is_Zero(flow_v3 V, flow_real Epsilon) {
	flow_bool Result = Flow_V3_Sq_Mag(V) <= Epsilon;
	return Result;
}

static size_t Flow_V3_Largest_Index(flow_v3 v) {
	size_t LargestIndex = Flow_Abs(v.x) > Flow_Abs(v.y) ? 0 : 1;
	return Flow_Abs(v.Data[LargestIndex]) > Flow_Abs(v.Data[2]) ? LargestIndex : 2;
}

static flow_real Flow_V3_Largest(flow_v3 v) {
	return v.Data[Flow_V3_Largest_Index(v)];
}

static flow_bool Flow_V3_Is_Nan(flow_v3 v) {
	return Flow_Is_Nan(v.x) || Flow_Is_Nan(v.y) || Flow_Is_Nan(v.z);
}

static flow_bool Flow_V3_Is_Close(flow_v3 A, flow_v3 B, flow_real ToleranceSq) {
	flow_bool Result = Flow_V3_Sq_Mag(Flow_V3_Sub_V3(B, A)) <= ToleranceSq;
	return Result;
}

static flow_real Flow_V3_Angle_Between(flow_v3 V1, flow_v3 V2) {
	flow_real Dot = Flow_V3_Dot(V1, V2);
	flow_real Length = Flow_V3_Mag(V1)*Flow_V3_Mag(V2);
	return Flow_ACos(Flow_Saturate(Dot / Length));
}

typedef struct {
	union {
		struct {
			flow_real x, y, z, w;
		};
		flow_v3 xyz;
	};
} flow_v4;

static flow_v4 Flow_V4_From_V3(flow_v3 xyz, flow_real w) {
	flow_v4 Result = { xyz.x, xyz.y, xyz.z, w };
	return Result;
}

static flow_real Flow_V4_Dot(flow_v4 a, flow_v4 b) {
	flow_real Result = { a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w };
	return Result;
}

static flow_quat Flow_Quat(flow_real x, flow_real y, flow_real z, flow_real w) {
	flow_quat Result = { x, y, z, w };
	return Result;
}

static flow_quat Flow_Quat_Identity() {
	flow_quat Result = Flow_Quat(0, 0, 0, 1);
	return Result;
}

static flow_quat Flow_Quat_From_V3_S(flow_v3 v, flow_real s) {
	flow_quat Result = { v.x, v.y, v.z, s };
	return Result;
}

static flow_quat Flow_Quat_Mul_S(flow_quat A, flow_real B) {
	flow_quat Result = Flow_Quat(A.x * B, A.y * B, A.z * B, A.w * B);
	return Result;
}

static flow_quat Flow_Quat_Axis_Angle(flow_v3 Axis, flow_real Angle) {
	flow_real c = Flow_Cos(Angle * flow_constant(0.5));
	flow_real s = Flow_Sin(Angle * flow_constant(0.5));
	return Flow_Quat_From_V3_S(Flow_V3_Mul_S(Axis, s), c);
}

static flow_real Flow_Quat_Dot(flow_quat A, flow_quat B) {
	flow_real Result = A.x * B.x + A.y * B.y + A.z * B.z + A.w * B.w;;
	return Result;
}

static flow_real Flow_Quat_Sq_Mag(flow_quat V) {
	flow_real Result = Flow_Quat_Dot(V, V);
	return Result;
}

static flow_real Flow_Quat_Mag(flow_quat V) {
	flow_real Result = Flow_Sqrt(Flow_Quat_Sq_Mag(V));
	return Result;
}

static flow_quat Flow_Quat_Norm(flow_quat V) {
	flow_real SqLength = Flow_Quat_Sq_Mag(V);
	if (Flow_Equal_Zero_Eps_Sq(SqLength)) {
		return Flow_Quat_Identity();
	}

	flow_real InvLength = flow_constant(1.0) / Flow_Sqrt(SqLength);
	return Flow_Quat_Mul_S(V, InvLength);
}

static flow_quat Flow_Quat_Mul_Quat(flow_quat a, flow_quat b) {
	flow_v3 v = Flow_V3_Add_V3(Flow_V3_Cross(a.v, b.v), Flow_V3_Add_V3(Flow_V3_Mul_S(a.v, b.s), Flow_V3_Mul_S(b.v, a.s)));
	flow_real s = a.s * b.s - Flow_V3_Dot(a.v, b.v);
	return Flow_Quat_From_V3_S(v, s);
}

static flow_bool Flow_Quat_Is_Nan(flow_quat Q) {
	return Flow_Is_Nan(Q.x) || Flow_Is_Nan(Q.y) || Flow_Is_Nan(Q.z) || Flow_Is_Nan(Q.w);
}

typedef struct {
	union {
		flow_real Data[9];
		flow_v3 Rows[3];
		struct {
			flow_v3 x, y, z;
		};
		struct {
			flow_real m00, m01, m02;
			flow_real m10, m11, m12;
			flow_real m20, m21, m22;
		};
	};
} flow_m3;

static flow_m3 Flow_M3_Scale(flow_v3 Scale) {
	flow_m3 Result = {
		Scale.x, 0, 0,
		0, Scale.y, 0,
		0, 0, Scale.z
	};
	return Result;
}

static flow_m3 Flow_M3_Scale_All(flow_real Scale) {
	flow_m3 Result = {
		Scale, 0, 0,
		0, Scale, 0,
		0, 0, Scale
	};
	return Result;
}

static flow_m3 Flow_M3_Diagonal(flow_real x, flow_real y, flow_real z) {
	flow_m3 Result = {
		x, 0, 0,
		0, y, 0,
		0, 0, z
	};
	return Result;
}

static flow_m3 Flow_M3_Mul_S(flow_m3* M, flow_real S) {
	flow_m3 Result = { 0 };
	for (size_t i = 0; i < 9; i++) {
		Result.Data[i] = M->Data[i]*S;
	}
	return Result;
}

static flow_m3 Flow_M3_Transpose(const flow_m3* M) {
	flow_m3 Result = {
		M->m00,
		M->m10,
		M->m20,

		M->m01,
		M->m11,
		M->m21,

		M->m02,
		M->m12,
		M->m22
	};

	return Result;
}

static flow_m3 Flow_M3_Mul_M3(const flow_m3* A, const flow_m3* B) {
	flow_m3 BTransposed = Flow_M3_Transpose(B);

	flow_m3 Result = {
		Flow_V3_Dot(A->Rows[0], BTransposed.Rows[0]),
		Flow_V3_Dot(A->Rows[0], BTransposed.Rows[1]),
		Flow_V3_Dot(A->Rows[0], BTransposed.Rows[2]),

		Flow_V3_Dot(A->Rows[1], BTransposed.Rows[0]),
		Flow_V3_Dot(A->Rows[1], BTransposed.Rows[1]),
		Flow_V3_Dot(A->Rows[1], BTransposed.Rows[2]),

		Flow_V3_Dot(A->Rows[2], BTransposed.Rows[0]),
		Flow_V3_Dot(A->Rows[2], BTransposed.Rows[1]),
		Flow_V3_Dot(A->Rows[2], BTransposed.Rows[2])
	};

	return Result;
}

static flow_m3 Flow_M3_From_Quat(flow_quat q) {
	flow_real qxqy = q.x * q.y;
	flow_real qwqz = q.w * q.z;
	flow_real qxqz = q.x * q.z;
	flow_real qwqy = q.w * q.y;
	flow_real qyqz = q.y * q.z;
	flow_real qwqx = q.w * q.x;
        	         
	flow_real qxqx = q.x * q.x;
	flow_real qyqy = q.y * q.y;
	flow_real qzqz = q.z * q.z;

	flow_m3 Result = { 0 };
	Result.x = Flow_V3(1 - 2*(qyqy + qzqz), 2*(qxqy + qwqz),     2*(qxqz - qwqy));
	Result.y = Flow_V3(2*(qxqy - qwqz),     1 - 2*(qxqx + qzqz), 2*(qyqz + qwqx));
	Result.z = Flow_V3(2*(qxqz + qwqy),     2*(qyqz - qwqx),     1 - 2*(qxqx + qyqy));
	return Result;
}

static flow_m3 Flow_M3_Axis_Angle(flow_v3 Axis, flow_real Angle) {
	flow_quat Quat = Flow_Quat_Axis_Angle(Axis, Angle);
	return Flow_M3_From_Quat(Quat);
}

static flow_m3 Flow_M3_Inverse(const flow_m3* M) {
	flow_real xSq = Flow_V3_Sq_Mag(M->x);
	flow_real ySq = Flow_V3_Sq_Mag(M->y);
	flow_real zSq = Flow_V3_Sq_Mag(M->z);

	flow_real sx = Flow_Equal_Zero_Eps_Sq(xSq) ? flow_constant(0.0) : flow_constant(1.0)/xSq;
	flow_real sy = Flow_Equal_Zero_Eps_Sq(ySq) ? flow_constant(0.0) : flow_constant(1.0)/ySq;
	flow_real sz = Flow_Equal_Zero_Eps_Sq(zSq) ? flow_constant(0.0) : flow_constant(1.0)/zSq;

	flow_v3 NewX = Flow_V3_Mul_S(M->x, sx);
	flow_v3 NewY = Flow_V3_Mul_S(M->y, sy);
	flow_v3 NewZ = Flow_V3_Mul_S(M->z, sz);

	flow_m3 Result = {
		NewX.x, NewY.x, NewZ.x,
		NewX.y, NewY.y, NewZ.y,
		NewX.z, NewY.z, NewZ.z
	};
	return Result;
}

static flow_v3 Flow_V3_Mul_M3(flow_v3 A, const flow_m3* B) {
	flow_m3 BTransposed = Flow_M3_Transpose(B);

	flow_v3 Result = {
		Flow_V3_Dot(A, BTransposed.Rows[0]),
		Flow_V3_Dot(A, BTransposed.Rows[1]),
		Flow_V3_Dot(A, BTransposed.Rows[2])
	};
	return Result;
}

static flow_m3 Flow_M3_Basis(flow_v3 Direction) {
	flow_v3 Z = Flow_V3_Norm(Direction);
	flow_v3 Up = Flow_V3(0, 1, 0);

	flow_real Diff = 1 - Flow_Abs(Flow_V3_Dot(Z, Up));
	if(Flow_Equal_Zero_Eps(Diff))
		Up = Flow_V3(0, 0, 1);

	flow_v3 X = Flow_V3_Negate(Flow_V3_Norm(Flow_V3_Cross(Z, Up)));
	flow_v3 Y = Flow_V3_Cross(Z, X);

	flow_m3 Result = { 0 };
	Result.x = X;
	Result.y = Y;
	Result.z = Z;

	return Result;
}

typedef struct {
	union {
		flow_real Data[16];
		flow_v4 Rows[4];

		struct {
			flow_v3 x; flow_real __unused0__;
			flow_v3 y; flow_real __unused1__;
			flow_v3 z; flow_real __unused2__;
			flow_v3 t; flow_real __unused3__;
		};

		struct {
			flow_real m00, m01, m02, m03;
			flow_real m10, m11, m12, m13;
			flow_real m20, m21, m22, m23;
			flow_real m30, m31, m32, m33;
		};
	};
} flow_m4;

static flow_m3 Flow_M4_Get_M3(const flow_m4* M) {
	flow_m3 Result = {
		M->m00,
		M->m01,
		M->m02,

		M->m10,
		M->m11,
		M->m12,

		M->m20,
		M->m21,
		M->m22,
	};
	return Result;
}

static flow_m4 Flow_M4_Transform(flow_v3 P, flow_quat Orientation) {
	flow_m3 M = Flow_M3_From_Quat(Orientation);
	flow_m4 Result = {
		M.m00, M.m01, M.m02, 0, 
		M.m10, M.m11, M.m12, 0, 
		M.m20, M.m21, M.m22, 0, 
		P.x, P.y, P.z, 1
	};
	return Result;
}

static flow_m4 Flow_M4_Inverse_Transform(flow_v3 P, flow_quat Orientation) {
	flow_m3 M = Flow_M3_From_Quat(Orientation);

	flow_real tx = -Flow_V3_Dot(P, M.x);
	flow_real ty = -Flow_V3_Dot(P, M.y);
	flow_real tz = -Flow_V3_Dot(P, M.z);

	flow_m4 Result = {
		M.m00, M.m10, M.m20, 0,
		M.m01, M.m11, M.m21, 0, 
		M.m02, M.m12, M.m22, 0, 
		tx, ty, tz, 1
	};
	return Result;
}

static flow_m4 Flow_M4_Transpose(const flow_m4* M) {
	flow_m4 Result = {
		M->m00,
		M->m10,
		M->m20,
		M->m30,
		      		
		M->m01,
		M->m11,
		M->m21,
		M->m31,
		      		
		M->m02,
		M->m12,
		M->m22,
		M->m32,
		      		
		M->m03,
		M->m13,
		M->m23,
		M->m33,
	};

	return Result;
}

static flow_v4 Flow_V4_Mul_M4(flow_v4 A, const flow_m4* B) {
	flow_m4 BTransposed = Flow_M4_Transpose(B);

	flow_v4 Result = {
		Flow_V4_Dot(A, BTransposed.Rows[0]),
		Flow_V4_Dot(A, BTransposed.Rows[1]),
		Flow_V4_Dot(A, BTransposed.Rows[2]),
		Flow_V4_Dot(A, BTransposed.Rows[3])
	};
	return Result;
}

static flow_m4 Flow_M4_Inverse(const flow_m4* M) {
	flow_real tx = -Flow_V3_Dot(M->t, M->x);
	flow_real ty = -Flow_V3_Dot(M->t, M->y);
	flow_real tz = -Flow_V3_Dot(M->t, M->z);

	flow_m4 Result = {
		M->m00, M->m10, M->m20, 0,
		M->m01, M->m11, M->m21, 0,
		M->m02, M->m12, M->m22, 0,
		tx,     ty,     tz,     1
	};

	return Result;
}

static flow_m4 Flow_M4_Mul_M4(const flow_m4* A, const flow_m4* B) {
	flow_m4 BTransposed = Flow_M4_Transpose(B);

	flow_m4 Result = { 0 };
	Result.Data[0] = Flow_V4_Dot(A->Rows[0], BTransposed.Rows[0]);
	Result.Data[1] = Flow_V4_Dot(A->Rows[0], BTransposed.Rows[1]);
	Result.Data[2] = Flow_V4_Dot(A->Rows[0], BTransposed.Rows[2]);
	Result.Data[3] = Flow_V4_Dot(A->Rows[0], BTransposed.Rows[3]);

	Result.Data[4] = Flow_V4_Dot(A->Rows[1], BTransposed.Rows[0]);
	Result.Data[5] = Flow_V4_Dot(A->Rows[1], BTransposed.Rows[1]);
	Result.Data[6] = Flow_V4_Dot(A->Rows[1], BTransposed.Rows[2]);
	Result.Data[7] = Flow_V4_Dot(A->Rows[1], BTransposed.Rows[3]);

	Result.Data[8]  = Flow_V4_Dot(A->Rows[2], BTransposed.Rows[0]);
	Result.Data[9]  = Flow_V4_Dot(A->Rows[2], BTransposed.Rows[1]);
	Result.Data[10] = Flow_V4_Dot(A->Rows[2], BTransposed.Rows[2]);
	Result.Data[11] = Flow_V4_Dot(A->Rows[2], BTransposed.Rows[3]);

	Result.Data[12] = Flow_V4_Dot(A->Rows[3], BTransposed.Rows[0]);
	Result.Data[13] = Flow_V4_Dot(A->Rows[3], BTransposed.Rows[1]);
	Result.Data[14] = Flow_V4_Dot(A->Rows[3], BTransposed.Rows[2]);
	Result.Data[15] = Flow_V4_Dot(A->Rows[3], BTransposed.Rows[3]);

	return Result;
}

static flow_m3 Flow_M3_From_M4(const flow_m4* M) {
	flow_m3 Result = { 0 };
	Result.x = M->x;	
	Result.y = M->y;	
	Result.z = M->z; 
	return Result;
}

static flow_v3 Flow_Get_Triangle_Centroid(flow_v3 P0, flow_v3 P1, flow_v3 P2) {
	flow_real Denominator = flow_constant(1.0/3.0);
	flow_v3 Result = Flow_V3_Mul_S(Flow_V3_Add_V3(Flow_V3_Add_V3(P0, P1), P2), Denominator);
	return Result;
}

#define FLOW_GJK_SUPPORT_FUNC(name) flow_v3 name(void* UserData, flow_v3 Direction)
typedef FLOW_GJK_SUPPORT_FUNC(flow_gjk_support_func);

typedef struct {
	void* 			  	   UserData;
	flow_gjk_support_func* SupportFunc;
} flow_gjk_support;

typedef struct {
	flow_v3 P;
} flow_gjk_point;

typedef struct {
	flow_real Radius;
} flow_gjk_radius;

typedef struct {
	flow_v3 CenterP;
	flow_real Radius;
} flow_gjk_sphere;

typedef struct {
	flow_v3 HalfExtent;
} flow_gjk_extent;

typedef struct {
	flow_v3 Min;
	flow_v3 Max;
} flow_gjk_aabb;

typedef struct {
	flow_gjk_support Support;
	flow_m4 Transform;
	flow_m3 InvTransform;
} flow_gjk_transform;

typedef struct {
	flow_real DistSq;
	flow_v3 P1;
	flow_v3 P2;
} flow_closest_points;

typedef struct {
	flow_bool Intersected;
	flow_v3 P1;
	flow_v3 P2;
	flow_v3 V;
} flow_penetration_test;

static flow_bool Flow_Get_Line_Barycentric_From_Origin(flow_v3 p0, flow_v3 p1, flow_real* OutU, flow_real* OutV) {
	flow_v3 ab = Flow_V3_Sub_V3(p1, p0);
	flow_real Denominator = Flow_V3_Sq_Mag(ab);
	if (Denominator < Flow_Sq(FLOW_EPSILON)) {
		//Degenerate line segment. Check which point is the closest
		if (Flow_V3_Sq_Mag(p0) < Flow_V3_Sq_Mag(p1)) {
			*OutU = flow_constant(1.0);
			*OutV = flow_constant(0.0);
		} else {
			*OutU = flow_constant(0.0);
			*OutV = flow_constant(1.0);
		}

		return flow_false;
	} else {
		*OutV = Flow_V3_Dot(Flow_V3_Negate(p0), ab) / Denominator;
		*OutU = flow_constant(1.0) - *OutV;
	}
	return flow_true;
}

static flow_bool Flow_Get_Triangle_Barycentric_From_Origin(flow_v3 p0, flow_v3 p1, flow_v3 p2, flow_real* OutU, flow_real* OutV, flow_real* OutW) {
	// With p being 0, we can improve the numerical stability by including the shortest edge of the triangle 
	// in the calculation. 
	// see: Real-Time Collision Detection - Christer Ericson (Section: Barycentric Coordinates)
	//First calculate the 3 edges
	flow_v3 e0 = Flow_V3_Sub_V3(p1, p0);
	flow_v3 e1 = Flow_V3_Sub_V3(p2, p0);
	flow_v3 e2 = Flow_V3_Sub_V3(p2, p1);
	flow_real d00 = Flow_V3_Sq_Mag(e0);
	flow_real d11 = Flow_V3_Sq_Mag(e1);
	flow_real d22 = Flow_V3_Sq_Mag(e2);
	if (d00 <= d22) {
		flow_real d01 = Flow_V3_Dot(e0, e1);
		flow_real Denominator = d00 * d11 - Flow_Sq(d01);
		if (Denominator < flow_constant(1.0e-12)) {
			//Degenerate triangle, get the line barycentric coordinate instead
			if (d00 > d11) {
				Flow_Get_Line_Barycentric_From_Origin(p0, p1, OutU, OutV);
				*OutW = flow_constant(0.0);
			} else {
				Flow_Get_Line_Barycentric_From_Origin(p0, p2, OutU, OutW);
				*OutV = flow_constant(0.0);
			}
			return flow_false;
		} else {
			flow_real a0 = Flow_V3_Dot(p0, e0);
			flow_real a1 = Flow_V3_Dot(p0, e1);
			*OutV = (d01 * a1-d11*a0) / Denominator;
			*OutW = (d01 * a0-d00*a1) / Denominator;
			*OutU = flow_constant(1.0) - *OutV - *OutW;
		}
	} else {
		flow_real d12 = Flow_V3_Dot(e1, e2);
		flow_real Denominator = d11 * d22 - Flow_Sq(d12);
		if (Denominator < flow_constant(1.0e-12)) {
			//Degenerate triangle, get the line barycentric coordinate instead
			if (d11 > d22) {
				Flow_Get_Line_Barycentric_From_Origin(p0, p2, OutU, OutW);
				*OutV = flow_constant(0.0);
			} else {
				Flow_Get_Line_Barycentric_From_Origin(p1, p2, OutV, OutW);
				*OutU = flow_constant(0.0);
			}
			return flow_false;
		} else {
			flow_real c1 = Flow_V3_Dot(p2, e1);
			flow_real c2 = Flow_V3_Dot(p2, e2);
			*OutU = (d22 * c1-d12*c2) / Denominator;
			*OutV = (d11 * c2-d12*c1) / Denominator;
			*OutW = flow_constant(1.0) - *OutU - *OutV;
		}
	}
	return flow_true;
}

static flow_v3 Flow_Closest_Point_From_Line_To_Origin_Set(flow_v3 p0, flow_v3 p1, flow_u32* OutSet) {
	flow_real u, v;
	Flow_Get_Line_Barycentric_From_Origin(p0, p1, &u, &v);
	if (v <= flow_constant(0.0)) {
		*OutSet = 0b0001;
		return p0;
	} else if (u <= flow_constant(0.0)) {
		*OutSet = 0b0010;
		return p1;
	} else {
		*OutSet = 0b0011;
		return Flow_V3_Add_V3(Flow_V3_Mul_S(p0, u), Flow_V3_Mul_S(p1, v));
	}
}

static flow_v3 Flow_Closest_Point_From_Triangle_To_Origin_Set(flow_v3 p0, flow_v3 p1, flow_v3 p2, flow_bool MustIncludeC, flow_u32* OutSet) {
	//First we need to check if our triangle is a degenerate triangle. We can detect this using the normal
	//but wee ideally want to calculate the normal  with the smallest precision loss possible. We check if 
	//edge ac is smaller than bc and if its not, we swap ac and bc
	//see: https://box2d.org/posts/2014/01/troublesome-triangle/
	flow_v3 ac = Flow_V3_Sub_V3(p2, p0);
	flow_v3 bc = Flow_V3_Sub_V3(p2, p1);
	flow_bool ShouldSwapAC = Flow_V3_Sq_Mag(bc) < Flow_V3_Sq_Mag(ac);
	flow_v3 a = ShouldSwapAC ? p2 : p0;
	flow_v3 c = ShouldSwapAC ? p0 : p2;
	flow_v3 b = p1;
	//Calculate the normal
	flow_v3 ab = Flow_V3_Sub_V3(b, a);
	ac = Flow_V3_Sub_V3(c, a);
	flow_v3 n = Flow_V3_Cross(ab, ac);
	flow_real NLenSq = Flow_V3_Sq_Mag(n);
	flow_u32 Set = 0;
	if (NLenSq < 1e-11f) {
		//Degenerate case on vertices and edges
		//Start with vertex C being the closest point
		Set = 0b0100;
		flow_v3 BestPoint = p2;
		flow_real BestDistSq = Flow_V3_Sq_Mag(p2);
		//A or B cannot be the closest if we must include c
		if (!MustIncludeC) {
			//Test if vertex A is closer
			flow_real ADistSq = Flow_V3_Sq_Mag(p0);
			if (ADistSq < BestDistSq) {
				Set = 0b0001;
				BestPoint = p0;
				BestDistSq = ADistSq;
			}
			//Test if vertex B is closer
			flow_real BDistSq = Flow_V3_Sq_Mag(p1);
			if (BDistSq < BestDistSq) {
				Set = 0b0010;
				BestPoint = p1;
				BestDistSq = BDistSq;
			}
		}
		//Check if the edge AC is closer
		flow_real ACDistSq = Flow_V3_Sq_Mag(ac);
		if (ACDistSq > Flow_Sq(FLOW_EPSILON)) {
			flow_real v = Flow_Clamp(0.0f, -Flow_V3_Dot(a, ac) / ACDistSq, 1.0f);
			flow_v3 q = Flow_V3_Add_V3(a, Flow_V3_Mul_S(ac, v));
			flow_real DistSq = Flow_V3_Sq_Mag(q);
			if (DistSq < BestDistSq) {
				Set = 0b0101;
				BestPoint = q;
				BestDistSq = DistSq;
			}
		}
		//Check if the edge BC is closer
		flow_v3 bc = Flow_V3_Sub_V3(p2, p1);
		flow_real BCDistSq = Flow_V3_Sq_Mag(bc);
		if (BCDistSq > Flow_Sq(FLOW_EPSILON)) {
			flow_real v = Flow_Clamp(0.0f, -Flow_V3_Dot(b, bc) / BCDistSq, 1.0f);
			flow_v3 q = Flow_V3_Add_V3(p1, Flow_V3_Mul_S(bc, v));
			flow_real DistSq = Flow_V3_Sq_Mag(q);
			if (DistSq < BestDistSq) {
				Set = 0b0110;
				BestPoint = q;
				BestDistSq = DistSq;
			}
		}
		//If we must include C then AB cannot be the closest
		if (!MustIncludeC) {
			//Check if the edge AB is closer
			flow_v3 ab = Flow_V3_Sub_V3(p1, p0);
			flow_real ABDistSq = Flow_V3_Sq_Mag(ab);
			if (ABDistSq > Flow_Sq(FLOW_EPSILON)) {
				flow_real v = Flow_Clamp(0.0f, -Flow_V3_Dot(p0, ab) / ABDistSq, 1.0f);
				flow_v3 q = Flow_V3_Add_V3(p0, Flow_V3_Mul_S(ab, v));
				flow_real DistSq = Flow_V3_Sq_Mag(q);
				if (DistSq < BestDistSq) {
					Set = 0b0011;
					BestPoint = q;
					BestDistSq = DistSq;
				}
			}
		}
		
		*OutSet = Set;
		return BestPoint;
	}
	//Normal triangle cases. Detect the 8 regions and see which feature includes the closest point
	//Vertex region A
	flow_v3 ap = Flow_V3_Negate(a);
	flow_real d1 = Flow_V3_Dot(ab, ap);
	flow_real d2 = Flow_V3_Dot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f) {
		*OutSet = ShouldSwapAC ? 0b0100 : 0b0001;
		return a; //BC: (1, 0, 0)
	}
	//Vertex region B
	flow_v3 bp = Flow_V3_Negate(p1);
	flow_real d3 = Flow_V3_Dot(ab, bp);
	flow_real d4 = Flow_V3_Dot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3) {
		*OutSet = 0b0010;
		return p1; //BC: (0, 1, 0)
	}
	//Edge region AB. If its the best region project the origin onto AB
	if (d1 * d4 <= d3 * d2 && d1 >= 0.0f && d3 <= 0.0f) {
		flow_real v = d1 / (d1 - d3);
		*OutSet = ShouldSwapAC ? 0b0110 : 0b0011;
		return Flow_V3_Add_V3(a, Flow_V3_Mul_S(ab, v));  //BC: (1-v, v, 0)
	}
	//Vertex region C
	flow_v3 cp = Flow_V3_Negate(c);
	flow_real d5 = Flow_V3_Dot(ab, cp);
	flow_real d6 = Flow_V3_Dot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6) {
		*OutSet = ShouldSwapAC ? 0b0001 : 0b0100;
		return c; //BC: (0, 0, 1)
	}
	//Edge region AC. If its the best region project the origin onto AC
	if (d5 * d2 <= d1 * d6 && d2 >= 0.0f && d6 <= 0.0f) {
		flow_real w = d2 / (d2 - d6);
		*OutSet = 0b0101;
		return Flow_V3_Add_V3(a, Flow_V3_Mul_S(ac, w)); //BC: (1-w, 0, w)
	}
	//Edge region BC. If its the best region project the origin onto BC
	flow_real d4_d3 = d4 - d3;
	flow_real d5_d6 = d5 - d6;
	if (d3 * d6 <= d5 * d4 && d4_d3 >= 0.0f && d5_d6 >= 0.0f) {
		flow_real w = d4_d3 / (d4_d3 + d5_d6);
		*OutSet = ShouldSwapAC ? 0b0011 : 0b0110;
		return Flow_V3_Add_V3(b, Flow_V3_Mul_S(Flow_V3_Sub_V3(c, b), w)); //BC: (0, 1-w, w)
	}
	//The triangle face is the best region. Project the origin onto the face
	*OutSet = 0b0111;
	return Flow_V3_Mul_S(Flow_V3_Mul_S(n, Flow_V3_Dot(Flow_V3_Add_V3(Flow_V3_Add_V3(a, b), c), n)), (1.0f / (3.0f * NLenSq)));
}

static void Flow_Check_If_Origin_Is_Outside_Of_Tetrahedron_Planes(flow_v3 p0, flow_v3 p1, flow_v3 p2, flow_v3 p3, flow_bool* Results) {
	// see: Real-Time Collision Detection - Christer Ericson (Section: Closest Point on Tetrahedron to Point)
	// With p = 0
	
	flow_v3 ab = Flow_V3_Sub_V3(p1, p0);
	flow_v3 ac = Flow_V3_Sub_V3(p2, p0);
	flow_v3 ad = Flow_V3_Sub_V3(p3, p0);
	flow_v3 bd = Flow_V3_Sub_V3(p3, p1);
	flow_v3 bc = Flow_V3_Sub_V3(p2, p1);
	flow_v3 ab_cross_ac = Flow_V3_Cross(ab, ac);
	flow_v3 ac_cross_ad = Flow_V3_Cross(ac, ad);
	flow_v3 ad_cross_ab = Flow_V3_Cross(ad, ab);
	flow_v3 bd_cross_bc = Flow_V3_Cross(bd, bc);
	// Get which side the origin is on for each plane 
	flow_real SignP[4] = {
		Flow_V3_Dot(p0, ab_cross_ac), //Plane ABC
		Flow_V3_Dot(p0, ac_cross_ad), //Plane ACD
		Flow_V3_Dot(p0, ad_cross_ab), //Plane ADB
		Flow_V3_Dot(p1, bd_cross_bc)  //Plane BDC
	};
	// Get the fourth component of the planes
	flow_real SignD[4] = {
		Flow_V3_Dot(ad, ab_cross_ac),
		Flow_V3_Dot(ab, ac_cross_ad),
		Flow_V3_Dot(ac, ad_cross_ab),
		-Flow_V3_Dot(ab, bd_cross_bc)
	};
	//Determine if the sign of all triangle components are the same. If not it is a degenerate
	//case where the origin is in front of all sides
	flow_u32 SignBits = 0;
	SignBits = ((((SignD[0] < 0) ? 0xFF : 0x00) << 0) |
		(((SignD[1] < 0) ? 0xFF : 0x00) << 8) |
		(((SignD[2] < 0) ? 0xFF : 0x00) << 16) |
		(((SignD[3] < 0) ? 0xFF : 0x00) << 24));
	switch (SignBits) {
		case 0x00000000: {
			Results[0] = SignP[0] >= -FLOW_EPSILON;
			Results[1] = SignP[1] >= -FLOW_EPSILON;
			Results[2] = SignP[2] >= -FLOW_EPSILON;
			Results[3] = SignP[3] >= -FLOW_EPSILON;
		} break;
		case 0xFFFFFFFF: {
			Results[0] = SignP[0] <= FLOW_EPSILON;
			Results[1] = SignP[1] <= FLOW_EPSILON;
			Results[2] = SignP[2] <= FLOW_EPSILON;
			Results[3] = SignP[3] <= FLOW_EPSILON;
		} break;
		default: {
			Results[0] = flow_true;
			Results[1] = flow_true;
			Results[2] = flow_true;
			Results[3] = flow_true;
		} break;
	}
}

static flow_v3 Flow_Closest_Point_From_Tetrahedron_To_Origin_Set(flow_v3 p0, flow_v3 p1, flow_v3 p2, flow_v3 p3, flow_bool MustIncludeD, flow_u32* OutSet) {
	// see: Real-Time Collision Detection - Christer Ericson (Section: Closest Point on Tetrahedron to Point)
	// With p = 0
	
	flow_u32 ClosestSet = 0b1111;
	flow_v3 BestPoint = Flow_V3_Zero();
	flow_real BestDistSq = FLOW_MAX_REAL;
	flow_bool OriginPlaneCheck[4];
	Flow_Check_If_Origin_Is_Outside_Of_Tetrahedron_Planes(p0, p1, p2, p3, OriginPlaneCheck);
	//Test against plane abc
	if (OriginPlaneCheck[0]) {
		if (MustIncludeD) {
			//If the closest point must include D then ABC cannot be closest but the closest point
			//cannot be an interior point either so we return A as closest point
			ClosestSet = 0b0001;
			BestPoint = p0;
		} else {
			//Test the face normally
			BestPoint = Flow_Closest_Point_From_Triangle_To_Origin_Set(p0, p1, p2, flow_false, &ClosestSet);
		}
		BestDistSq = Flow_V3_Sq_Mag(BestPoint);
	}
	//Test against plane acd
	if (OriginPlaneCheck[1]) {
		flow_u32 TempSet;
		flow_v3 TempPoint = Flow_Closest_Point_From_Triangle_To_Origin_Set(p0, p2, p3, MustIncludeD, &TempSet);
		flow_real DistSq = Flow_V3_Sq_Mag(TempPoint);
		if (DistSq < BestDistSq) {
			BestDistSq = DistSq;
			BestPoint = TempPoint;
			ClosestSet = (TempSet & 0b0001) + ((TempSet & 0b0110) << 1);
		}
	}

	//Test against plane adb
	if (OriginPlaneCheck[2]) {
		flow_u32 TempSet;
		flow_v3 TempPoint = Flow_Closest_Point_From_Triangle_To_Origin_Set(p0, p1, p3, MustIncludeD, &TempSet);
		flow_real DistSq = Flow_V3_Sq_Mag(TempPoint);
		if (DistSq < BestDistSq) {
			BestDistSq = DistSq;
			BestPoint = TempPoint;
			ClosestSet = (TempSet & 0b0011) + ((TempSet & 0b0100) << 1);
		}
	}

	//Test against plane bdc
	if (OriginPlaneCheck[3]) {
		flow_u32 TempSet;
		flow_v3 TempPoint = Flow_Closest_Point_From_Triangle_To_Origin_Set(p1, p2, p3, MustIncludeD, &TempSet);
		flow_real DistSq = Flow_V3_Sq_Mag(TempPoint);
		if (DistSq < BestDistSq) {
			BestDistSq = DistSq;
			BestPoint = TempPoint;
			ClosestSet = TempSet << 1;
		}
	}

	*OutSet = ClosestSet;
	return BestPoint;
}

static flow_v3 Flow_GJK_Get_Support(flow_gjk_support* Support, flow_v3 Direction) {
	flow_v3 Result = Support->SupportFunc(Support->UserData, Direction);
	return Result;
}

static FLOW_GJK_SUPPORT_FUNC(Flow_GJK_Origin_Support) {
	
	return Flow_V3_Zero();
}

static FLOW_GJK_SUPPORT_FUNC(Flow_GJK_Point_Support) {
	flow_gjk_point* Point = (flow_gjk_point *)UserData;
	return Point->P;
}

static FLOW_GJK_SUPPORT_FUNC(Flow_GJK_Radius_Support) {
	flow_gjk_radius* Radius = (flow_gjk_radius *)UserData;
	flow_real Length = Flow_V3_Mag(Direction);
	return Length > flow_constant(0) ? Flow_V3_Mul_S(Direction, Radius->Radius / Length) : Flow_V3_Zero();
}

static FLOW_GJK_SUPPORT_FUNC(Flow_GJK_Sphere_Support) {
	flow_gjk_sphere* Sphere = (flow_gjk_sphere *)UserData;
	flow_real Length = Flow_V3_Mag(Direction);
	return Length > flow_constant(0) ? Flow_V3_Add_V3(Sphere->CenterP, Flow_V3_Mul_S(Direction, Sphere->Radius / Length)) : Sphere->CenterP;
}

static FLOW_GJK_SUPPORT_FUNC(Flow_GJK_Extent_Support) {
	flow_gjk_extent* Extent = (flow_gjk_extent *)UserData;

	flow_v3 Result = {
		Direction.x < flow_constant(0.0) ? -Extent->HalfExtent.x : Extent->HalfExtent.x,
		Direction.y < flow_constant(0.0) ? -Extent->HalfExtent.y : Extent->HalfExtent.y,
		Direction.z < flow_constant(0.0) ? -Extent->HalfExtent.z : Extent->HalfExtent.z
	};

	return Result;
}

static FLOW_GJK_SUPPORT_FUNC(Flow_GJK_AABB_Support) {
	flow_gjk_aabb* AABB = (flow_gjk_aabb *)UserData;

	flow_v3 Result = {
		Direction.x < flow_constant(0.0) ? AABB->Min.x : AABB->Max.x,
		Direction.y < flow_constant(0.0) ? AABB->Min.y : AABB->Max.y,
		Direction.z < flow_constant(0.0) ? AABB->Min.z : AABB->Max.z
	};

	return Result;
}

static FLOW_GJK_SUPPORT_FUNC(Flow_GJK_Transform_Support) {
	flow_gjk_transform* Transform = (flow_gjk_transform *)UserData;
	flow_v3 P = Flow_GJK_Get_Support(&Transform->Support, Flow_V3_Mul_M3(Direction, &Transform->InvTransform));
	return Flow_V4_Mul_M4(Flow_V4_From_V3(P, flow_constant(1.0)), &Transform->Transform).xyz;
}

static flow_gjk_support Flow_GJK_Make_Support(flow_gjk_support_func* Func, void* UserData) {
	flow_gjk_support Result = {
		UserData,
		Func
	};
	return Result;
}

static flow_gjk_support Flow_GJK_Origin() {
	flow_gjk_support Support = Flow_GJK_Make_Support(Flow_GJK_Origin_Support, NULL);
	return Support;
}

static flow_gjk_support Flow_GJK_Point(flow_arena* Arena, flow_v3 P) {
	flow_gjk_point* Point = Flow_Arena_Push_Struct(Arena, flow_gjk_point);
	Point->P = P;
	
	flow_gjk_support Support = Flow_GJK_Make_Support(Flow_GJK_Point_Support, Point);
	return Support;
}

static flow_gjk_support Flow_GJK_Radius(flow_arena* Arena, flow_real Radius) {
	flow_gjk_radius* GJKRadius = Flow_Arena_Push_Struct(Arena, flow_gjk_radius);
	GJKRadius->Radius = Radius;

	flow_gjk_support Support = Flow_GJK_Make_Support(Flow_GJK_Radius_Support, GJKRadius);
	return Support;
}

static flow_gjk_support Flow_GJK_Sphere(flow_arena* Arena, flow_v3 P, flow_real Radius) {
	flow_gjk_sphere* Sphere = Flow_Arena_Push_Struct(Arena, flow_gjk_sphere);
	Sphere->Radius = Radius;
	Sphere->CenterP = P;

	flow_gjk_support Support = Flow_GJK_Make_Support(Flow_GJK_Sphere_Support, Sphere);
	return Support;
}

static flow_gjk_support Flow_GJK_Extent(flow_arena* Arena, flow_v3 HalfExtent) {
	flow_gjk_extent* Extent = Flow_Arena_Push_Struct(Arena, flow_gjk_extent);
	Extent->HalfExtent = HalfExtent;

	flow_gjk_support Support = Flow_GJK_Make_Support(Flow_GJK_Extent_Support, Extent);
	return Support;
}

static flow_gjk_support Flow_GJK_AABB(flow_arena* Arena, flow_v3 Min, flow_v3 Max) {
	flow_gjk_aabb* AABB = Flow_Arena_Push_Struct(Arena, flow_gjk_aabb);
	AABB->Min = Min;
	AABB->Max = Max;

	flow_gjk_support Support = Flow_GJK_Make_Support(Flow_GJK_AABB_Support, AABB);
	return Support;
}

static flow_gjk_support Flow_GJK_Transform(flow_arena* Arena, flow_gjk_support InnerSupport, const flow_m4* Matrix) {
	flow_gjk_transform* Transform = Flow_Arena_Push_Struct(Arena, flow_gjk_transform);
	Transform->Support = InnerSupport;
	Transform->Transform = *Matrix;

	flow_m3 M = Flow_M4_Get_M3(&Transform->Transform);
	Transform->InvTransform = Flow_M3_Transpose(&M);

	flow_gjk_support Support = Flow_GJK_Make_Support(Flow_GJK_Transform_Support, Transform);
	return Support;
}

typedef struct {
	flow_u32 Count;
	flow_v3 P[4];
	flow_v3 A[4];
	flow_v3 B[4];
} flow_gjk_simplex;

static flow_real Flow_GJK_Max_P_Dist_Sq(flow_gjk_simplex* Simplex) {
	flow_real BestDistSq = Flow_V3_Sq_Mag(Simplex->P[0]);
	for (flow_u32 i = 1; i < Simplex->Count; i++) {
		BestDistSq = Flow_Max(BestDistSq, Flow_V3_Sq_Mag(Simplex->P[i]));
	}
	return BestDistSq;
}

static void Flow_GJK_Update_P(flow_gjk_simplex* Simplex, flow_u32 Set) {
	flow_u32 Count = 0;
	for (flow_u32 i = 0; i < Simplex->Count; i++) {
		if ((Set & (1 << i)) != 0) {
			Simplex->P[Count] = Simplex->P[i];
			Count++;
		}
	}
	Simplex->Count = Count;
}

static void Flow_GJK_Update_PAB(flow_gjk_simplex* Simplex, flow_u32 Set) {
	flow_u32 Count = 0;
	for (flow_u32 i = 0; i < Simplex->Count; i++) {
		if ((Set & (1 << i)) != 0) {
			Simplex->P[Count] = Simplex->P[i];
			Simplex->A[Count] = Simplex->A[i];
			Simplex->B[Count] = Simplex->B[i];
			Count++;
		}
	}
	Simplex->Count = Count;
}

static void Flow_GJK_Calculate_Closest_Points(flow_gjk_simplex* Simplex, flow_v3* OutA, flow_v3* OutB) {
	switch (Simplex->Count) {
		case 1: {
			*OutA = Simplex->A[0];
			*OutB = Simplex->B[0];
		} break;

		case 2: {
			flow_real u, v;
			Flow_Get_Line_Barycentric_From_Origin(Simplex->P[0], Simplex->P[1], &u, &v);
			*OutA = Flow_V3_Add_V3(Flow_V3_Mul_S(Simplex->A[0], u), Flow_V3_Mul_S(Simplex->A[1], v));
			*OutB = Flow_V3_Add_V3(Flow_V3_Mul_S(Simplex->B[0], u), Flow_V3_Mul_S(Simplex->B[1], v));
		} break;

		case 3: {
			flow_real u, v, w;
			Flow_Get_Triangle_Barycentric_From_Origin(Simplex->P[0], Simplex->P[1], Simplex->P[2], &u, &v, &w);
			*OutA = Flow_V3_Add_V3(Flow_V3_Add_V3(Flow_V3_Mul_S(Simplex->A[0], u), Flow_V3_Mul_S(Simplex->A[1], v)), Flow_V3_Mul_S(Simplex->A[2], w));
			*OutB = Flow_V3_Add_V3(Flow_V3_Add_V3(Flow_V3_Mul_S(Simplex->B[0], u), Flow_V3_Mul_S(Simplex->B[1], v)), Flow_V3_Mul_S(Simplex->B[2], w));
		} break;
	}
}

static flow_bool Flow_GJK_Get_Closest_Point(flow_gjk_simplex* Simplex, flow_real PrevDistSq, flow_v3* OutV, flow_real* OutDistSq, 
								   	  		flow_u32* OutSet, flow_bool IncludeLastFeature) {
	flow_u32 Set;
	flow_v3 V = { 0, 0, 0 };

	switch (Simplex->Count) {
		case 1: {
			//Single point case
			Set = 0b0001;
			V = Simplex->P[0];
		} break;

		case 2: {
			//Line segment case
			V = Flow_Closest_Point_From_Line_To_Origin_Set(Simplex->P[0], Simplex->P[1], &Set);
		} break;

		case 3: {
			//Triangle case
			V = Flow_Closest_Point_From_Triangle_To_Origin_Set(Simplex->P[0], Simplex->P[1], Simplex->P[2], IncludeLastFeature, &Set);
		} break;

		case 4: {
			//Tetrahedron case
			V = Flow_Closest_Point_From_Tetrahedron_To_Origin_Set(Simplex->P[0], Simplex->P[1], Simplex->P[2], Simplex->P[3], IncludeLastFeature, &Set);
		} break;

		default: {
			Flow_Assert(flow_false);
			return flow_false; //Prevent compiler warnings
		} break;
	}

	//Make sure the closest point is actually closer than the previous point. If its not, we are not converging
	//and should exit out of the gjk iterations
	flow_real DistSq = Flow_V3_Sq_Mag(V);
	if (DistSq <= PrevDistSq) {
		*OutV = V;
		*OutDistSq = DistSq;
		*OutSet = Set;
		return flow_true;
	}

	return flow_false;
}

static flow_bool Flow_GJK_Intersects(flow_gjk_support* SupportA, flow_gjk_support* SupportB, flow_real Tolerance, flow_v3* InOutV) {
	flow_real ToleranceSq = Flow_Sq(Tolerance);

	flow_gjk_simplex Simplex = { 0 };

	//Used to make sure v is converging closer to the origin
	flow_real PrevDistSq = FLOW_MAX_REAL;

	for (;;) {
		
		//Iteration always starts with getting the support points on each object
		//in the direction of V. Then subtracting the two (aka the Minkowski difference)
		flow_v3 a = Flow_GJK_Get_Support(SupportA, *InOutV);
		flow_v3 b = Flow_GJK_Get_Support(SupportB, Flow_V3_Negate(*InOutV));
		flow_v3 p = Flow_V3_Sub_V3(a, b);

		//Check if the support point is in the opposite direction of p, if it is we found a 
		//separating axis
		if (Flow_V3_Dot(*InOutV, p) < 0.0f) {
			return flow_false;
		}

		//Add P to the simplex
		Simplex.P[Simplex.Count] = p;
		Simplex.Count++;

		flow_real DistSq;
		flow_u32 Set;

		//Get the new closest point 
		if (!Flow_GJK_Get_Closest_Point(&Simplex, PrevDistSq, InOutV, &DistSq, &Set, flow_true)) {
			return flow_false;
		}

		//If there are 4 points in the simplex, the origin is inside the tetrahedron and 
		//we have intersected
		if (Set == 0xf) {
			*InOutV = Flow_V3_Zero();
			return flow_true;
		}

		//If V is extremely close to zero, we consider this a collision
		if (DistSq <= ToleranceSq) {
			*InOutV = Flow_V3_Zero();
			return flow_true;
		}

		//If V is very small compared to the length of P in the simplex, this is also a collision
		if (DistSq <= FLOW_EPSILON*Flow_GJK_Max_P_Dist_Sq(&Simplex)) {
			*InOutV = Flow_V3_Zero();
			return flow_true;
		}

		*InOutV = Flow_V3_Negate(*InOutV);

		//If V is not converging and going closer to the origin on every iteration, we are not changing
		//and will not find a valid solution. Therefore there is no coliision possible
		Flow_Assert(PrevDistSq >= DistSq);
		if (PrevDistSq - DistSq <= FLOW_EPSILON*PrevDistSq) {
			return flow_false;
		}

		PrevDistSq = DistSq;

		Flow_GJK_Update_P(&Simplex, Set);
	}
}

static flow_closest_points Flow_GJK_Get_Closest_Points(flow_gjk_support* SupportA, flow_gjk_support* SupportB, 
													   flow_real Tolerance, flow_real MaxDistSq, flow_v3* InOutV, 
											   	  	   flow_gjk_simplex* OutSimplex) {
	flow_real ToleranceSq = Flow_Sq(Tolerance);

	flow_gjk_simplex Simplex = { 0 };

	flow_real DistSq = Flow_V3_Sq_Mag(*InOutV);
	flow_real PrevDistSq = FLOW_MAX_REAL;

	for (;;) {
		//Iteration always starts with getting the support points on each object
		//in the direction of V. Then subtracting the two (aka the Minkowski difference)
		flow_v3 a = Flow_GJK_Get_Support(SupportA, *InOutV);
		flow_v3 b = Flow_GJK_Get_Support(SupportB, Flow_V3_Negate(*InOutV));
		flow_v3 p = Flow_V3_Sub_V3(a, b);

		
		//Check if we have a separating that is greater than MaxDistSq. If we do we can terminate early
		flow_real Dot = Flow_V3_Dot(*InOutV, p);
		if (Dot < 0.0f && Flow_Sq(Dot) > DistSq * MaxDistSq) {
			flow_closest_points Result;
			Result.DistSq = FLOW_MAX_REAL;
			return Result;
		}

		//Add P and the support points to the simplex. Support points are used to calculate the closest
		//points at the end of the algorithm
		Simplex.P[Simplex.Count] = p;
		Simplex.A[Simplex.Count] = a;
		Simplex.B[Simplex.Count] = b;
		Simplex.Count++;

		flow_u32 Set;
		if (!Flow_GJK_Get_Closest_Point(&Simplex, PrevDistSq, InOutV, &DistSq, &Set, flow_true)) {
			Simplex.Count--;
			break;
		}

		//If there are 4 points in the simplex, the origin is inside the tetrahedron and 
		//we have intersected
		if (Set == 0xF) {
			*InOutV = Flow_V3_Zero();
			DistSq = 0.0f;
			break;
		}

		Flow_GJK_Update_PAB(&Simplex, Set);

		//If V is extremely close to zero, we consider this a collision
		if (DistSq <= ToleranceSq) {
			*InOutV = Flow_V3_Zero();
			DistSq = 0.0f;
			break;
		}

		//If V is very small compared to the length of P in the simplex, this is also a collision
		if (DistSq <= FLOW_EPSILON*Flow_GJK_Max_P_Dist_Sq(&Simplex)) {
			*InOutV = Flow_V3_Zero();
			DistSq = 0.0f;
			break;
		}

		*InOutV = Flow_V3_Negate(*InOutV);

		//If V is not converging and going closer to the origin on every iteration, we are not changing
		//and will not find a valid solution. Therefore there is no coliision possible
		Flow_Assert(PrevDistSq >= DistSq);
		if (PrevDistSq - DistSq <= FLOW_EPSILON*PrevDistSq) {
			break;
		}

		PrevDistSq = DistSq;
	}

	flow_closest_points Result = { 0 };
	Result.DistSq = DistSq;
	Flow_GJK_Calculate_Closest_Points(&Simplex, &Result.P1, &Result.P2);

	if (OutSimplex) *OutSimplex = Simplex;

	return Result;
}

static flow_penetration_test Flow_GJK_Penetration_Test(flow_gjk_support* SupportA, flow_real RadiusA, flow_gjk_support* SupportB, 
													   flow_real RadiusB, flow_real Tolerance, flow_gjk_simplex* OutSimplex, 
													   flow_real* OutDistSq) {
	flow_real Radius = RadiusA + RadiusB;
	flow_real RadiusSq = Flow_Sq(Radius);

	flow_v3 V = { 1, 0, 0 };
	flow_closest_points ClosestPoints = Flow_GJK_Get_Closest_Points(SupportA, SupportB, Tolerance, RadiusSq, &V, OutSimplex);

	if (ClosestPoints.DistSq > RadiusSq) {
		//No collision
		flow_penetration_test Result = { 0 };
		return Result;
	}

	flow_penetration_test Result = { 0 };
	Result.Intersected = flow_true;

	*OutDistSq = ClosestPoints.DistSq;
	
	if (ClosestPoints.DistSq > 0.0f) {
		//Collision within the convex radius
		flow_real Dist = Flow_Sqrt(ClosestPoints.DistSq);
		Result.P2 = Flow_V3_Add_V3(ClosestPoints.P1, Flow_V3_Mul_S(V, RadiusA / Dist));
		Result.P1 = Flow_V3_Sub_V3(ClosestPoints.P2, Flow_V3_Mul_S(V, RadiusB / Dist));
		Result.V  = Flow_V3_Sub_V3(Result.P2, Result.P1);
	}

	return Result;
}

#define FLOW_EPA_MAX_POINTS 128
#define FLOW_EPA_MAX_TRIANGLES 256
#define FLOW_EPA_MIN_TRIANGLE_AREA 1.0e-10f
#define FLOW_EPA_BARYCENTRIC_EPSILON 0.001f
#define FLOW_EPA_MAX_EDGES 128

typedef struct {
	flow_u32  Count;
	flow_v3 P[FLOW_EPA_MAX_POINTS];
} flow_epa_points;

typedef struct {
	flow_epa_points Points;
	flow_v3 A[FLOW_EPA_MAX_POINTS];
	flow_v3 B[FLOW_EPA_MAX_POINTS];
} flow_epa_support;

typedef struct flow_epa_triangle flow_epa_triangle;

typedef struct {
	flow_epa_triangle* NeighborTriangle;
	flow_u32 		  NeighborEdge;
	flow_u32 		  StartVtx;
} flow_epa_edge;

struct flow_epa_triangle {
	flow_epa_edge Edges[3];
	flow_v3 	 	 Normal;
	flow_v3 	 	 Centroid;
	flow_real  	 DistSq;
	flow_real 	 Lambda[2];
	flow_bool8 	 	 IsLambdaRelativeTo0;
	flow_bool8 	 	 Removed;
	flow_bool8 	 	 InQueue;
	flow_bool8 		 IsClosestPointInterior;

#ifdef __cplusplus
	flow_epa_triangle() {}
#endif
};

typedef union flow_epa_triangle_pool_entry flow_epa_triangle_pool_entry;

union flow_epa_triangle_pool_entry { 
	flow_epa_triangle  			 Triangle;
	flow_epa_triangle_pool_entry* NextFree;

#ifdef __cplusplus
	flow_epa_triangle_pool_entry() {}
#endif
};

typedef struct {
	flow_epa_triangle_pool_entry  Triangles[FLOW_EPA_MAX_TRIANGLES];
	flow_epa_triangle_pool_entry* NextFree;
	flow_u64 					  MaxUsed;
} flow_epa_triangle_pool;

typedef struct {
	size_t 			   Count;
	flow_epa_triangle* Triangles[FLOW_EPA_MAX_TRIANGLES];
} flow_epa_heap;

typedef struct {
	flow_epa_heap      Heap;
} flow_epa_triangle_queue;

typedef struct {
	flow_epa_points*        Points;
	flow_epa_triangle_pool  TrianglePool;
	flow_epa_triangle_queue TriangleQueue;
} flow_epa_convex_hull;

typedef struct {
	flow_u32 		  Count;
	flow_epa_triangle* Ptr[FLOW_EPA_MAX_TRIANGLES];
} flow_epa_triangle_list;

typedef struct {
	flow_u32 	 Count;
	flow_epa_edge Ptr[FLOW_EPA_MAX_EDGES];
} flow_epa_edge_list;

#define Flow_EPA_Heap_Parent(i) ((i - 1) / 2)
// to get index of left child of node at index i 
#define Flow_EPA_Heap_Left(i) (2 * i + 1)
// to get index of right child of node at index i 
#define Flow_EPA_Heap_Right(i) (2 * i + 2)

static void Flow_EPA_Heap_Swap(flow_epa_heap* Heap, size_t Index0, size_t Index1) {	
	flow_epa_triangle* Tmp = Heap->Triangles[Index0];
	Heap->Triangles[Index0] = Heap->Triangles[Index1];
	Heap->Triangles[Index1] = Tmp;
}

static flow_bool Flow_EPA_Compare_Triangles(flow_epa_triangle* TriangleA, flow_epa_triangle* TriangleB) {
	return TriangleB->DistSq > TriangleA->DistSq;
}

static void Flow_EPA_Heap_Maxify(flow_epa_heap* Heap) {
	size_t Index = 0;
	size_t Largest = Index;
	do {
		size_t Left = Flow_EPA_Heap_Left(Index);
		size_t Right = Flow_EPA_Heap_Right(Index);

		size_t Count = Heap->Count;

		if (Left < Count && Flow_EPA_Compare_Triangles(Heap->Triangles[Left], Heap->Triangles[Index])) {
			Largest = Left;
		}

		if (Right < Count && Flow_EPA_Compare_Triangles(Heap->Triangles[Right], Heap->Triangles[Largest])) {
			Largest = Right;
		}

		if (Largest != Index) {
			Flow_EPA_Heap_Swap(Heap, Index, Largest);
			Index = Largest;
		}
		
	} while (Largest != Index);
}

static void Flow_EPA_Heap_Push(flow_epa_heap* Heap, flow_epa_triangle* Triangle) {
	Flow_Assert(Heap->Count < FLOW_EPA_MAX_TRIANGLES);
	size_t Index = Heap->Count++;
	Heap->Triangles[Index] = Triangle;

	//If the parent of the binary heap is smaller, we need to fixup the structure
	while (Index != 0 && !Flow_EPA_Compare_Triangles(Heap->Triangles[Flow_EPA_Heap_Parent(Index)], 
													 Heap->Triangles[Index])) {
		Flow_EPA_Heap_Swap(Heap, Index, Flow_EPA_Heap_Parent(Index));
		Index = Flow_EPA_Heap_Parent(Index);
	}
}

static flow_epa_triangle* Flow_EPA_Heap_Pop(flow_epa_heap* Heap) {
	Flow_Assert(Heap->Count);
	if (Heap->Count == 1) {
		Heap->Count--;
		return Heap->Triangles[0];
	}

	Flow_EPA_Heap_Swap(Heap, 0, Heap->Count-1);
	flow_epa_triangle* Result = Heap->Triangles[Heap->Count - 1];
	Heap->Count--;
	Flow_EPA_Heap_Maxify(Heap);
	return Result;
}

static flow_v3 Flow_EPA_Support_Push(flow_epa_support* SupportPoints, flow_gjk_support* SupportA, flow_gjk_support* SupportB, 
							 		 flow_v3 Direction, flow_u32* OutIndex) {
	flow_v3 a = Flow_GJK_Get_Support(SupportA, Direction);
	flow_v3 b = Flow_GJK_Get_Support(SupportB, Flow_V3_Negate(Direction));
	flow_v3 p = Flow_V3_Sub_V3(a, b);

	flow_u32 Index = SupportPoints->Points.Count++;

	*OutIndex = Index;
	SupportPoints->Points.P[Index] = p;
	SupportPoints->A[Index] = a;
	SupportPoints->B[Index] = b;
	return p;
}

static void Flow_EPA_Support_Pop(flow_epa_support* SupportPoints) {
	Flow_Assert(SupportPoints->Points.Count > 0);
	SupportPoints->Points.Count--;
}

static flow_epa_edge* Flow_EPA_Triangle_Get_Next_Edge(flow_epa_triangle* Triangle, flow_u32 EdgeIdx) {
	flow_epa_edge* Edge = Triangle->Edges + ((EdgeIdx + 1) % 3);
	return Edge;
}

static void Flow_EPA_Link_Triangles(flow_epa_triangle* T0, flow_u32 EdgeIdx0, flow_epa_triangle* T1, flow_u32 EdgeIdx1) {
	Flow_Assert(EdgeIdx0 < 3);
	Flow_Assert(EdgeIdx1 < 3);
	flow_epa_edge* Edge0 = T0->Edges + EdgeIdx0;
	flow_epa_edge* Edge1 = T1->Edges + EdgeIdx1;

	Flow_Assert(Edge0->NeighborTriangle == NULL);
	Flow_Assert(Edge1->NeighborTriangle == NULL);

	Flow_Assert(Edge0->StartVtx == Flow_EPA_Triangle_Get_Next_Edge(T1, EdgeIdx1)->StartVtx);
	Flow_Assert(Edge1->StartVtx == Flow_EPA_Triangle_Get_Next_Edge(T0, EdgeIdx0)->StartVtx);

	Edge0->NeighborTriangle = T1;
	Edge0->NeighborEdge = EdgeIdx1;
	Edge1->NeighborTriangle = T0;
	Edge1->NeighborEdge = EdgeIdx0;
}

static void Flow_EPA_Triangle_Init(flow_epa_triangle* Triangle, flow_u32 VtxIdx0, flow_u32 VtxIdx1, flow_u32 VtxIdx2, flow_v3* Positions) {
	Flow_Memset(Triangle, 0, sizeof(flow_epa_triangle));
	Triangle->DistSq = FLOW_MAX_REAL;

	Triangle->Edges[0].StartVtx = VtxIdx0;
	Triangle->Edges[1].StartVtx = VtxIdx1;
	Triangle->Edges[2].StartVtx = VtxIdx2;

	flow_v3 V0 = Positions[VtxIdx0];
	flow_v3 V1 = Positions[VtxIdx1];
	flow_v3 V2 = Positions[VtxIdx2];

	//Calculate the centroid
	Triangle->Centroid = Flow_Get_Triangle_Centroid(V0, V1, V2);

	flow_v3 e0 = Flow_V3_Sub_V3(V1, V0);
	flow_v3 e1 = Flow_V3_Sub_V3(V2, V0);
	flow_v3 e2 = Flow_V3_Sub_V3(V2, V1);

	//Calculating the most accurate normal requires taking the cross product on the two shortest edges.
	//Figure out which edge, e1 or e2 is shorter than the other
	flow_real e1_dot_e1 = Flow_V3_Sq_Mag(e1); //Dot product with itself is just a sq mag
	flow_real e2_dot_e2 = Flow_V3_Sq_Mag(e2);

	if (e1_dot_e1 < e2_dot_e2) {
		//We select edges e0 and e1
		Triangle->Normal = Flow_V3_Cross(e0, e1);

		//Check if the triangle is a degenerate
		flow_real NormalSqLen = Flow_V3_Sq_Mag(Triangle->Normal);
		if (NormalSqLen > FLOW_EPA_MIN_TRIANGLE_AREA) {

			//Determine distance between triangle and origin
			flow_real CDotN = Flow_V3_Dot(Triangle->Centroid, Triangle->Normal);
			Triangle->DistSq = Flow_Abs(CDotN)*CDotN / NormalSqLen;

			//Compute the cloest point to the origin using barycentric coordinates
			flow_real e0_dot_e0 = Flow_V3_Sq_Mag(e0);
			flow_real e0_dot_e1 = Flow_V3_Dot(e0, e1);
			flow_real Determinant = e0_dot_e0 * e1_dot_e1 - Flow_Sq(e0_dot_e1);
			
			//If Determinant == 0 then the system is linearly dependent and the triangle is degenerate
			if (Determinant > 0) { 
				flow_real v0_dot_e0 = Flow_V3_Dot(V0, e0);
				flow_real v0_dot_e1 = Flow_V3_Dot(V0, e1);
				flow_real l0 = (e0_dot_e1 * v0_dot_e1 - e1_dot_e1 * v0_dot_e0) / Determinant;
				flow_real l1 = (e0_dot_e1 * v0_dot_e0 - e0_dot_e0 * v0_dot_e1) / Determinant;
				Triangle->Lambda[0] = l0;
				Triangle->Lambda[1] = l1;
				Triangle->IsLambdaRelativeTo0 = flow_true;

				//Check if the closest point is interior to the triangle. Our convex hull might contain
				//coplanar triangles and only one will have the origin as the interior point. This triangle
				//is best to use when calculating contract points for accuracy, so we will prioritize add these 
				//triangles into the priority queue
				if (l0 > -FLOW_EPA_BARYCENTRIC_EPSILON && l1 > -FLOW_EPA_BARYCENTRIC_EPSILON && l0 + l1 < 1.0f + FLOW_EPA_BARYCENTRIC_EPSILON)
					Triangle->IsClosestPointInterior = flow_true;
			}
		}
	} else {
		//We select edges e0 and e2
		Triangle->Normal = Flow_V3_Cross(e0, e2);
		//Check if the triangle is a degenerate
		flow_real NormalSqLen = Flow_V3_Sq_Mag(Triangle->Normal);
		if (NormalSqLen > FLOW_EPA_MIN_TRIANGLE_AREA) {
			//Determine distance between triangle and origin
			flow_real CDotN = Flow_V3_Dot(Triangle->Centroid, Triangle->Normal);
			Triangle->DistSq = Flow_Abs(CDotN)*CDotN / NormalSqLen;

			//Compute the cloest point to the origin using barycentric coordinates
			flow_real e0_dot_e0 = Flow_V3_Sq_Mag(e0);
			flow_real e0_dot_e2 = Flow_V3_Dot(e0, e2);
			flow_real Determinant = e0_dot_e0 * e2_dot_e2 - Flow_Sq(e0_dot_e2);
			//If Determinant == 0 then the system is linearly dependent and the triangle is degenerate
			if (Determinant > 0) { 
				flow_real v1_dot_e0 = Flow_V3_Dot(V1, e0);
				flow_real v1_dot_e2 = Flow_V3_Dot(V1, e2);
				flow_real l0 = (e2_dot_e2 * v1_dot_e0 - e0_dot_e2 * v1_dot_e2) / Determinant;
				flow_real l1 = (e0_dot_e2 * v1_dot_e0 - e0_dot_e0 * v1_dot_e2) / Determinant;
				Triangle->Lambda[0] = l0;
				Triangle->Lambda[1] = l1;
				Triangle->IsLambdaRelativeTo0 = flow_false;

				//Check if the closest point is interior to the triangle. Our convex hull might contain
				//coplanar triangles and only one will have the origin as the interior point. This triangle
				//is best to use when calculating contract points for accuracy, so we will prioritize add these 
				//triangles into the priority queue
				if (l0 > -FLOW_EPA_BARYCENTRIC_EPSILON && l1 > -FLOW_EPA_BARYCENTRIC_EPSILON && l0 + l1 < 1.0f + FLOW_EPA_BARYCENTRIC_EPSILON)
					Triangle->IsClosestPointInterior = flow_true;
			}
		}
	}
}

static flow_bool Flow_EPA_Triangle_Is_Facing(flow_epa_triangle* Triangle, flow_v3 P) {
	Flow_Assert(!Triangle->Removed);
	flow_bool Result = Flow_V3_Dot(Triangle->Normal, Flow_V3_Sub_V3(P, Triangle->Centroid)) > 0.0f;
	return Result;
}

static flow_bool Flow_EPA_Triangle_Is_Facing_Origin(flow_epa_triangle* Triangle) {
	Flow_Assert(!Triangle->Removed);
	flow_bool Result = Flow_V3_Dot(Triangle->Normal, Triangle->Centroid) < 0.0f;
	return Result;
}

static flow_epa_triangle* Flow_EPA_Triangle_Pool_Create_Triangle(flow_epa_triangle_pool* Pool, flow_u32 VtxIdx0, flow_u32 VtxIdx1, flow_u32 VtxIdx2, flow_v3* Positions) {
	flow_epa_triangle* Triangle;
	if (Pool->NextFree != NULL) {
		Triangle = &Pool->NextFree->Triangle;
		Pool->NextFree = Pool->NextFree->NextFree;
	} else {
		if (Pool->MaxUsed >= FLOW_EPA_MAX_TRIANGLES) {
			return NULL;
		}

		Triangle = &Pool->Triangles[Pool->MaxUsed].Triangle;
		Pool->MaxUsed++;
	}

	Flow_EPA_Triangle_Init(Triangle, VtxIdx0, VtxIdx1, VtxIdx2, Positions);
	return Triangle;
}

static void Flow_EPA_Triangle_Pool_Free_Triangle(flow_epa_triangle_pool* Pool, flow_epa_triangle* Triangle) {
	flow_epa_triangle_pool_entry* Entry = (flow_epa_triangle_pool_entry *)Triangle;
	Entry->NextFree = Pool->NextFree;
	Pool->NextFree = Entry;
}

static void Flow_EPA_Triangle_Queue_Push(flow_epa_triangle_queue* TriangleQueue, flow_epa_triangle* Triangle) {
	Triangle->InQueue = flow_true;
	Flow_EPA_Heap_Push(&TriangleQueue->Heap, Triangle);
}

static void Flow_EPA_Triangle_List_Push(flow_epa_triangle_list* TriangleList, flow_epa_triangle* Triangle) {
	TriangleList->Ptr[TriangleList->Count++] = Triangle;
}

static void Flow_EPA_Edge_List_Push(flow_epa_edge_list* EdgeList, flow_epa_edge* Edge) {
	EdgeList->Ptr[EdgeList->Count++] = *Edge;
}

static flow_epa_triangle* Flow_EPA_Triangle_Queue_Peek(flow_epa_triangle_queue* TriangleQueue) {
	flow_epa_triangle* Result = TriangleQueue->Heap.Triangles[0];
	return Result;
}

static flow_epa_triangle* Flow_EPA_Triangle_Queue_Pop(flow_epa_triangle_queue* TriangleQueue) {
	flow_epa_triangle* Result = Flow_EPA_Heap_Pop(&TriangleQueue->Heap);
	return Result;
}

static flow_bool Flow_EPA_Triangle_Queue_Empty(flow_epa_triangle_queue* TriangleQueue) {
	return TriangleQueue->Heap.Count == 0;
}

static flow_epa_triangle* Flow_EPA_Convex_Hull_Create_Triangle(flow_epa_convex_hull* ConvexHull, flow_u32 VtxIdx0, flow_u32 VtxIdx1, flow_u32 VtxIdx2) {
	flow_epa_triangle* Triangle = Flow_EPA_Triangle_Pool_Create_Triangle(&ConvexHull->TrianglePool, VtxIdx0, VtxIdx1, VtxIdx2, ConvexHull->Points->P);
	return Triangle;
}

static void Flow_EPA_Convex_Hull_Free_Triangle(flow_epa_convex_hull* ConvexHull, flow_epa_triangle* Triangle) {
#ifdef FLOW_DEBUG
	Flow_Assert(Triangle->Removed);
	for (flow_u32 i = 0; i < 3; i++) {
		flow_epa_edge* Edge = Triangle->Edges + i;
		Flow_Assert(Edge->NeighborTriangle == NULL);
	}
#endif
	Flow_EPA_Triangle_Pool_Free_Triangle(&ConvexHull->TrianglePool, Triangle);
}

static void Flow_EPA_Convex_Hull_Unlink_Triangle(flow_epa_convex_hull* ConvexHull, flow_epa_triangle* Triangle) {
	for (flow_u32 i = 0; i < 3; i++) {
		flow_epa_edge* Edge = Triangle->Edges + i;
		if (Edge->NeighborTriangle != NULL) {
			flow_epa_edge* NeighborEdge = Edge->NeighborTriangle->Edges + Edge->NeighborEdge;

			Flow_Assert(NeighborEdge->NeighborTriangle == Triangle);
			Flow_Assert(NeighborEdge->NeighborEdge == i);

			//Unlink
			NeighborEdge->NeighborTriangle = NULL;
			Edge->NeighborTriangle = NULL;
		}
	}

	//If the triangle is not in the queue we can delete it now
	if (!Triangle->InQueue) {
		Flow_EPA_Convex_Hull_Free_Triangle(ConvexHull, Triangle);
	}
}

static flow_bool Flow_EPA_Convex_Hull_Find_Edge(flow_epa_convex_hull* ConvexHull, flow_epa_triangle* FacingTriangle, flow_v3 P, flow_epa_edge_list* OutEdges) {
	//Given a triangle that faces the vertex P, find the edges of the triangles that do not face P and flag the
	//triangles for removal

	Flow_Assert(OutEdges->Count == 0);
	Flow_Assert(Flow_EPA_Triangle_Is_Facing(FacingTriangle, P));
	
	//Flag facing triangle for removal
	FacingTriangle->Removed = flow_true;

	//Recurse to find and remove triangles that are not facing the vertex P
	typedef struct {
		flow_epa_triangle* Triangle;
		flow_u32 		  EdgeIdx;
		flow_s32 		  Iteration;
	} stack_entry;

	flow_s32 CurrentStackIndex = 0;
	stack_entry Stack[FLOW_EPA_MAX_EDGES];

	//Start with the facing triangle 
	Stack[CurrentStackIndex].Triangle = FacingTriangle;
	Stack[CurrentStackIndex].EdgeIdx = 0;
	Stack[CurrentStackIndex].Iteration = -1;

	flow_s64 NextExpectedStartIdx = -1;

	for (;;) {
		stack_entry* CurrentEntry = Stack + CurrentStackIndex;
		if (++CurrentEntry->Iteration >= 3) {
			//Triangle should be removed now
			Flow_EPA_Convex_Hull_Unlink_Triangle(ConvexHull, CurrentEntry->Triangle);

			//Pop from the stack
			CurrentStackIndex--;
			if (CurrentStackIndex < 0)
				break;
		} else {
			flow_epa_edge* Edge = CurrentEntry->Triangle->Edges + ((CurrentEntry->EdgeIdx + CurrentEntry->Iteration) % 3);
			flow_epa_triangle* Neighbor = Edge->NeighborTriangle;
			if (Neighbor && !Neighbor->Removed) {
				if (Flow_EPA_Triangle_Is_Facing(Neighbor, P)) {
					//Triangle needs to be removed
					Neighbor->Removed = flow_true;

					CurrentStackIndex++;
					Flow_Assert(CurrentStackIndex < FLOW_EPA_MAX_EDGES);
					stack_entry* NewEntry = Stack + CurrentStackIndex;
					NewEntry->Triangle = Neighbor;
					NewEntry->EdgeIdx = Edge->NeighborEdge;
					NewEntry->Iteration = 0;
				} else {
					//If edge doesn't connect to the previous edge, we have found a very degenerate edge case and
					//will not be able to add the point to the convex hull
					if (Edge->StartVtx != NextExpectedStartIdx && NextExpectedStartIdx != -1)
						return flow_false;

					NextExpectedStartIdx = Neighbor->Edges[Edge->NeighborEdge].StartVtx;
					Flow_EPA_Edge_List_Push(OutEdges, Edge);
				}
			}
		}
	}

	Flow_Assert(OutEdges->Count == 0 || OutEdges->Ptr[0].StartVtx == NextExpectedStartIdx);
	return OutEdges->Count >= 3;
}

static flow_bool Flow_EPA_Convex_Hull_Add_Point(flow_epa_convex_hull* ConvexHull, flow_epa_triangle* Triangle, flow_u32 VtxIdx, flow_real MaxDistSq, flow_epa_triangle_list* NewTriangles) {
	flow_v3 Position = ConvexHull->Points->P[VtxIdx];

	//FInd the edges in the hull that are not facing the new vertex

	flow_epa_edge_list Edges = { 0 };
	if (!Flow_EPA_Convex_Hull_Find_Edge(ConvexHull, Triangle, Position, &Edges))
		return flow_false;

	//Create new triangles from the edges

	flow_u32 NumEdges = Edges.Count;
	for (flow_u32 i = 0; i < NumEdges; i++) {
		flow_epa_triangle* NewTriangle = Flow_EPA_Convex_Hull_Create_Triangle(ConvexHull, Edges.Ptr[i].StartVtx, 
																	Edges.Ptr[(i+1)%NumEdges].StartVtx,
																	VtxIdx);
		if (!NewTriangle) return flow_false;

		Flow_EPA_Triangle_List_Push(NewTriangles, NewTriangle);

		//Check if we should put this triangle in the priority queue
		if ((NewTriangle->IsClosestPointInterior && NewTriangle->DistSq < MaxDistSq) || NewTriangle->DistSq < 0.0f) {
			Flow_EPA_Triangle_Queue_Push(&ConvexHull->TriangleQueue, NewTriangle);
		}
	}

	//Link the edges
	for (flow_u32 i = 0; i < NumEdges; i++) {
		Flow_EPA_Link_Triangles(NewTriangles->Ptr[i], 0, Edges.Ptr[i].NeighborTriangle, Edges.Ptr[i].NeighborEdge);
		Flow_EPA_Link_Triangles(NewTriangles->Ptr[i], 1, NewTriangles->Ptr[(i + 1) % NumEdges], 2);
	}

	return flow_true;
}

static flow_epa_triangle* Flow_EPA_Convex_Hull_Find_Facing_Triangle(flow_epa_convex_hull* ConvexHull, flow_v3 P, flow_real* OutDistSq) {
	flow_epa_triangle* Result = NULL;
	flow_real BestDistSq = 0.0f;

	flow_epa_triangle_queue* TriangleQueue = &ConvexHull->TriangleQueue;
	for (flow_u64 i = 0; i < TriangleQueue->Heap.Count; i++) {
		flow_epa_triangle* Triangle = TriangleQueue->Heap.Triangles[i];
		if (!Triangle->Removed) {
			flow_real Dot = Flow_V3_Dot(Triangle->Normal, Flow_V3_Sub_V3(P, Triangle->Centroid));
			if (Dot > 0.0f) {
				flow_real DistSq = Flow_Sq(Dot) / Flow_V3_Sq_Mag(Triangle->Normal);
				if (DistSq > BestDistSq) {
					Result = Triangle;
					BestDistSq = DistSq;
				}
			}
		}
	}

	*OutDistSq = BestDistSq;
	return Result;
}

static flow_penetration_test Flow_EPA_Penetration_Test_With_Simplex(flow_gjk_support* SupportA, flow_gjk_support* SupportB, flow_real Tolerance, flow_gjk_simplex* Simplex) {
	flow_penetration_test Result = { 0 };

	//Copy the simplex into the epa support points
	flow_epa_support SupportPoints = { { Simplex->Count } };
	Flow_Memcpy(SupportPoints.Points.P, Simplex->P, Simplex->Count * sizeof(flow_v3));
	Flow_Memcpy(SupportPoints.A, Simplex->A, Simplex->Count * sizeof(flow_v3));
	Flow_Memcpy(SupportPoints.B, Simplex->B, Simplex->Count * sizeof(flow_v3));

	//The simplex should contain the origin, (GJK wouldn't report a collision otherwise), 
	//in cases where the simplex contains less than 3 points, we need to find extra support points
	switch (SupportPoints.Points.Count) {
		case 1: {
			//1 vertex, which must be the origin
			Flow_EPA_Support_Pop(&SupportPoints);

			//Add four points around the origin to form a tetrahedron around the origin

			flow_u32 I0, I1, I2, I3;
			Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Flow_V3( 0,  1,  0), &I0);
			Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Flow_V3(-1, -1, -1), &I1);
			Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Flow_V3( 1, -1, -1), &I2);
			Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Flow_V3( 0, -1,  1), &I3);

			Flow_Assert(I0 == 0);
			Flow_Assert(I1 == 1);
			Flow_Assert(I2 == 2);
			Flow_Assert(I3 == 3);
		} break;

		case 2: {
			//2 vertices which form a line that contains the origin

			//Create 3 extra vertices by taking a perpendicular axis and rotating it 120 degree increments
			flow_v3 Axis = Flow_V3_Norm(Flow_V3_Sub_V3(SupportPoints.Points.P[1], SupportPoints.Points.P[0]));
			flow_m3 Rotation = Flow_M3_Axis_Angle(Axis, Flow_To_Radians(120.0f));

			flow_v3 Dir1 = Flow_V3_Norm(Flow_V3_Get_Perp(Axis));
			flow_v3 Dir2 = Flow_V3_Mul_M3(Dir1, &Rotation);
			flow_v3 Dir3 = Flow_V3_Mul_M3(Dir2, &Rotation);

			flow_u32 I0, I1, I2;
			Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Dir1, &I0);
			Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Dir2, &I1);
			Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Dir3, &I2);

			Flow_Assert(I0 == 2);
			Flow_Assert(I1 == 3);
			Flow_Assert(I2 == 4);
		} break;

		case 3:
		case 4: {
			//Noop and do nothing. We have enough vertices already
		} break;

		Flow_Invalid_Default_Case;
	}

	Flow_Assert(SupportPoints.Points.Count >= 3);

	//Build the initial convex hull out of the simplex that contains the origin
	flow_epa_convex_hull ConvexHull = { &SupportPoints.Points };

	//Start with the first three vertices and create triangles back to back
	flow_epa_triangle* T1 = Flow_EPA_Convex_Hull_Create_Triangle(&ConvexHull, 0, 1, 2);
	flow_epa_triangle* T2 = Flow_EPA_Convex_Hull_Create_Triangle(&ConvexHull, 0, 2, 1);

	Flow_EPA_Link_Triangles(T1, 0, T2, 2);
	Flow_EPA_Link_Triangles(T1, 1, T2, 1);
	Flow_EPA_Link_Triangles(T1, 2, T2, 0);

	Flow_EPA_Triangle_Queue_Push(&ConvexHull.TriangleQueue, T1);
	Flow_EPA_Triangle_Queue_Push(&ConvexHull.TriangleQueue, T2);

	for (flow_u32 i = 3; i < SupportPoints.Points.Count; i++) {
		//Any additional points can get added to the hull. Make sure to add to the triangle
		//that is facing the most to the point

		flow_real DistSq;
		flow_epa_triangle* Triangle = Flow_EPA_Convex_Hull_Find_Facing_Triangle(&ConvexHull, SupportPoints.Points.P[i], &DistSq);
		if (Triangle != NULL) {
			flow_epa_triangle_list NewTriangles = { 0 };
			if (!Flow_EPA_Convex_Hull_Add_Point(&ConvexHull, Triangle, i, FLOW_MAX_REAL, &NewTriangles)) {
				//If we can't add a point, assume no collision. This can happen if the shapes touch
				//at one point, and its alright to not report a collision then
				return Result;
			}
		}
	}

	//After we created the initial hull, we need to make sure it includes the origin. The simplex
	//contained the origin, thus we know the hull must contain the origin at some point

	//Loop until we know the hull contains the origin
	for (;;) {
		flow_epa_triangle* Triangle = Flow_EPA_Triangle_Queue_Peek(&ConvexHull.TriangleQueue);
		
		//If a triangle was removed we can free it now. We don't free removed triangles immediately
		//since that means we would need to rebuild the binary heap
		if (Triangle->Removed) {
			Flow_EPA_Triangle_Queue_Pop(&ConvexHull.TriangleQueue);

			//If we ran out of triangles, there must be extremely little penetration and we can 
			//report no collision
			if (Flow_EPA_Triangle_Queue_Empty(&ConvexHull.TriangleQueue)) {
				return Result;
			}

			Flow_EPA_Convex_Hull_Free_Triangle(&ConvexHull, Triangle);
			continue;
		}

		//If the triangle closest to the origin is zero or positive, we know that the origin is in
		//the hull and can continue to the main algorithm
		if (Triangle->DistSq >= 0.0f)
			break;

		Flow_EPA_Triangle_Queue_Pop(&ConvexHull.TriangleQueue);

		//Add the support point to get the origin in the convex hull
		flow_u32 NewIndex;
		flow_v3 P = Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Triangle->Normal, &NewIndex);

		//Add the new point to the hull
		flow_epa_triangle_list NewTriangles = { 0 };
		if (!Flow_EPA_Triangle_Is_Facing(Triangle, P) || !Flow_EPA_Convex_Hull_Add_Point(&ConvexHull, Triangle, NewIndex, FLOW_MAX_REAL, &NewTriangles)) {
			//If we fail, terminate and report no collision
			return Result;
		}

		//Triangle can be safely removed, new support point is closer to the origin
		Flow_Assert(Triangle->Removed);
		Flow_EPA_Convex_Hull_Free_Triangle(&ConvexHull, Triangle);

		if (Flow_EPA_Triangle_Queue_Empty(&ConvexHull.TriangleQueue) || SupportPoints.Points.Count >= FLOW_EPA_MAX_POINTS) {
			//If we run out of points, there must be little penetration and we can report no collision
			return Result;
		}
	}

	/// - A: Calculate the closest point to the origin for all triangles of the hull and take the closest one
	/// - Calculate a new support point (of the Minkowski sum) in this direction and add this point to the convex hull
	/// - This will remove all faces that are facing the new point and will create new triangles to fill up the hole
	/// - Loop to A until no closer point found
	/// - The closest point indicates the position / direction of least penetration

	flow_real BestDistSq = FLOW_MAX_REAL;
	flow_epa_triangle* LastTriangle = NULL;

	flow_bool FlipSign = flow_false;

	do {
		//Get the closest triangle to the origin
		flow_epa_triangle* Triangle = Flow_EPA_Triangle_Queue_Pop(&ConvexHull.TriangleQueue);

		//To prevent rebuilding the heap, pop removed triangles now
		if (Triangle->Removed) {
			Flow_EPA_Convex_Hull_Free_Triangle(&ConvexHull, Triangle);
			continue;
		}

		//If we are not getting closer to the origin, the closest point has been found
		if (Triangle->DistSq >= BestDistSq) {
			break;
		}

		//Free the last triangle 
		if (LastTriangle != NULL) {
			Flow_EPA_Convex_Hull_Free_Triangle(&ConvexHull, LastTriangle);
		}
		LastTriangle = Triangle;

		//Calculate the new support point to add to the hull in the direction of the closest triangle
		flow_u32 NewIndex;
		flow_v3 p = Flow_EPA_Support_Push(&SupportPoints, SupportA, SupportB, Triangle->Normal, &NewIndex);

		//Project the point onto the contact normal
		flow_real Dot = Flow_V3_Dot(p, Triangle->Normal);

		//This shouldn't happen in practice, but its possible to have a separating axis in theory
		//and therefore no collision. Just in case, report no collision.
		if (Dot < 0.0f) {
			return Result;
		}

		//Get the sq dist to the support point (along direction of normal)
		flow_real DistSq = Flow_Sq(Dot) / Flow_V3_Sq_Mag(Triangle->Normal);

		//break if the error is small enough
		if (DistSq - Triangle->DistSq < Triangle->DistSq*Tolerance) {
			break;
		}

		BestDistSq = Flow_Min(BestDistSq, DistSq);

		//Due to numerical imprecision, we can technically have the triangle not facing the support point.
		//At this point we've converged and can just return
		if (!Flow_EPA_Triangle_Is_Facing(Triangle, p)) {
			break;
		}

		//Add support point to the hull, if it can't break out 
		flow_epa_triangle_list NewTriangles = { 0 };
		if (!Flow_EPA_Convex_Hull_Add_Point(&ConvexHull, Triangle, NewIndex, BestDistSq, &NewTriangles)) {
			break;
		}

		//Sometimes we build an invalid hull due to numerical imprecision. We break if this happens
		flow_bool IsInvalid = flow_false;
		for (flow_u32 i = 0; i < NewTriangles.Count; i++) {
			flow_epa_triangle* Triangle = NewTriangles.Ptr[i];
			if (Flow_EPA_Triangle_Is_Facing_Origin(Triangle)) {
				IsInvalid = flow_true;
				break;
			}
		}

		if (IsInvalid) {
			//When an invalid hull is generated (origin lies on the wrong side of the triangle)
			//we need to perform another check to see if the penetration of the triangle in the opposite 
			//direction of the normal is smaller than in the normal direction. If so we need to 
			//flip the sign of the penetration vector
			flow_v3 P2 = Flow_V3_Sub_V3(Flow_GJK_Get_Support(SupportA, Flow_V3_Negate(Triangle->Normal)), 
										Flow_GJK_Get_Support(SupportB, Triangle->Normal));
			flow_real Dot2 = -Flow_V3_Dot(Triangle->Normal, P2);
			if (Dot2 < Dot)
				FlipSign = flow_true;
			break;
		}
	} while (!Flow_EPA_Triangle_Queue_Empty(&ConvexHull.TriangleQueue) && SupportPoints.Points.Count < FLOW_EPA_MAX_POINTS);

	//If the hull was a plane, there is no penentration and report no collision
	if (LastTriangle == NULL) {
		return Result;
	}

	//Get the penetration vector. Calculated by getting the vector from the origin to the closest point
	//on the triangle.
	flow_v3 V = Flow_V3_Mul_S(LastTriangle->Normal, Flow_V3_Dot(LastTriangle->Centroid, LastTriangle->Normal) / Flow_V3_Sq_Mag(LastTriangle->Normal));
	
	//If the penetration is near zero, we treat this as no penetration and report no collision
	if (Flow_V3_Is_Zero(V, flow_constant(1.0e-12))) {
		return Result;
	}

	Result.Intersected = flow_true;

	if (FlipSign) {
		V = Flow_V3_Negate(V);
	}

	//Contact point generation
	flow_v3 a0 = SupportPoints.A[LastTriangle->Edges[0].StartVtx];
	flow_v3 a1 = SupportPoints.A[LastTriangle->Edges[1].StartVtx];
	flow_v3 a2 = SupportPoints.A[LastTriangle->Edges[2].StartVtx];

	flow_v3 b0 = SupportPoints.B[LastTriangle->Edges[0].StartVtx];
	flow_v3 b1 = SupportPoints.B[LastTriangle->Edges[1].StartVtx];
	flow_v3 b2 = SupportPoints.B[LastTriangle->Edges[2].StartVtx];

	if (LastTriangle->IsLambdaRelativeTo0)
	{
		//P1 is the reference point
		Result.P2 = Flow_V3_Add_V3(a0, Flow_V3_Add_V3(Flow_V3_Mul_S(Flow_V3_Sub_V3(a1, a0), LastTriangle->Lambda[0]), 
													  Flow_V3_Mul_S(Flow_V3_Sub_V3(a2, a0), LastTriangle->Lambda[1])));
		Result.P1 = Flow_V3_Add_V3(b0, Flow_V3_Add_V3(Flow_V3_Mul_S(Flow_V3_Sub_V3(b1, b0), LastTriangle->Lambda[0]), 
													  Flow_V3_Mul_S(Flow_V3_Sub_V3(b2, b0), LastTriangle->Lambda[1])));
	}
	else
	{
		//P2 is the reference point
		Result.P2 = Flow_V3_Add_V3(a1, Flow_V3_Add_V3(Flow_V3_Mul_S(Flow_V3_Sub_V3(a0, a1), LastTriangle->Lambda[0]), 
													  Flow_V3_Mul_S(Flow_V3_Sub_V3(a2, a1), LastTriangle->Lambda[1])));
		Result.P1 = Flow_V3_Add_V3(b1, Flow_V3_Add_V3(Flow_V3_Mul_S(Flow_V3_Sub_V3(b0, b1), LastTriangle->Lambda[0]), 
													  Flow_V3_Mul_S(Flow_V3_Sub_V3(b2, b1), LastTriangle->Lambda[1])));
	}
	Result.V = V;

	return Result;
}

static flow_penetration_test Flow_EPA_Penetration_Test(flow_gjk_support* SupportA, flow_gjk_support* SupportB, flow_real CollisionTolerance,
													   flow_real PenetrationTolerance) {
	flow_gjk_simplex Simplex;

	flow_real OutDist;
	flow_penetration_test Result = Flow_GJK_Penetration_Test(SupportA, flow_constant(0.0), SupportB, flow_constant(0.0), CollisionTolerance, &Simplex, &OutDist);
	if (Result.Intersected) {
		if (OutDist == flow_constant(0.0)) {
			Result = Flow_EPA_Penetration_Test_With_Simplex(SupportA, SupportB, PenetrationTolerance, &Simplex);
		}
	}
	return Result;
}


typedef struct {
	flow_body_id BodyA;
	flow_body_id BodyB;
	flow_v3 P1;
	flow_v3 P2;
	flow_v3 V;
} flow_penetration;

typedef struct {
	size_t Capacity;
	size_t Count;
	flow_penetration* Ptr;
} flow_penetration_array;

typedef struct {
	flow_v3 P1;
	flow_v3 P2;
} flow_collision_point;


#define FLOW_MAX_COLLISION_POINT_COUNT 64
typedef struct {
	flow_u32 Count;
	flow_collision_point Points[FLOW_MAX_COLLISION_POINT_COUNT];
} flow_collision_point_array;

typedef struct {
	flow_v3  LocalP1;
	flow_v3  LocalP2;
} flow_contact_point;

typedef struct {
	flow_bool Active;
    flow_v3   R1Axis;
    flow_v3   R2Axis;
    flow_v3   InvInertiaR1Axis;
    flow_v3   InvInertiaR2Axis;
    flow_real EffectiveMass;
    flow_real Bias;
    flow_real TotalLambda;
} flow_contact_constraint;

typedef struct {
	flow_contact_point 	    ContactPoint;
	flow_contact_constraint NormalConstraint;
	flow_contact_constraint TangentConstraints[2];
} flow_contact;

typedef struct {
	flow_body* 	 Body1;
	flow_body* 	 Body2;
	flow_real	 InvMass1;
	flow_real 	 InvMass2;
	flow_v3 	 Normal;
	flow_v3 	 Tangents[2];
	flow_u32     ContactCount;
	flow_real 	 Friction;
	flow_contact Contacts[4];
} flow_collision_constraint;

typedef struct {
	size_t Capacity;
	size_t Count;
	flow_collision_constraint* Ptr;
} flow_collision_constraint_array;

struct flow_system {
	flow_allocator Allocator;
	flow_arena TempArena;
	flow_body_pool BodyPool;
	
	flow_v3 Gravity;
	flow_f64 dt;

	flow_penetration_array PenetrationArray;
	flow_collision_constraint_array CollisionConstraintArray;
	
#ifdef FLOW_DEBUG_RENDERING
	flow_debug_renderer DebugRenderer;
#endif
};

#ifdef FLOW_DEBUG_RENDERING

static void Flow_Debug_Clear_(flow_system* System) {
	flow_debug_renderer* DebugRenderer = &System->DebugRenderer;
	DebugRenderer->CmdCount = 0;
}

static flow_debug_cmd* Flow_Debug_Push_Cmd(flow_system* System, flow_debug_cmd_type Type) {
	flow_debug_renderer* DebugRenderer = &System->DebugRenderer;
	if (DebugRenderer->CmdCount == DebugRenderer->CmdCapacity) {
		size_t NewCapacity = Max(DebugRenderer->CmdCapacity*2, 32);
		flow_debug_cmd* NewCmds = (flow_debug_cmd*)Flow_Allocate_Memory(&System->Allocator, NewCapacity * sizeof(flow_debug_cmd));

		if (DebugRenderer->Cmds) {
			Flow_Memcpy(NewCmds, DebugRenderer->Cmds, sizeof(flow_debug_cmd)*DebugRenderer->CmdCapacity);
			Flow_Free_Memory(&System->Allocator, DebugRenderer->Cmds);
		}

		DebugRenderer->Cmds = NewCmds;
		DebugRenderer->CmdCapacity = NewCapacity;
	}

	flow_debug_cmd* Cmd = DebugRenderer->Cmds + DebugRenderer->CmdCount++;
	Cmd->Type = Type;
	return Cmd;
}

static void Flow_Debug_Record_Penetration_(flow_system* System, flow_body* BodyA, flow_body* BodyB, flow_v3 P1, flow_v3 P2, flow_v3 V) {
	flow_debug_renderer* DebugRenderer = &System->DebugRenderer;
	flow_debug_cmd* Cmd = Flow_Debug_Push_Cmd(System, FLOW_DEBUG_CMD_PENETRATION);

	Cmd->Penetration.BodyA = Flow_Body_Pool_Get_ID(&System->BodyPool, BodyA);
	Cmd->Penetration.BodyB = Flow_Body_Pool_Get_ID(&System->BodyPool, BodyB);
	Cmd->Penetration.P1 = P1;
	Cmd->Penetration.P2 = P2;
	Cmd->Penetration.V = V;
}

#define Flow_Debug_Clear(...) Flow_Debug_Clear_(__VA_ARGS__)
#define Flow_Debug_Record_Penetration(...) Flow_Debug_Record_Penetration_(__VA_ARGS__)
#else
#define Flow_Debug_Clear(...)
#define Flow_Debug_Record_Penetration(...)
#endif

static void Flow_Body_Add_Rotation_Step(flow_body* Body, flow_v3 Delta) {
	flow_real DeltaLen = Flow_V3_Mag(Delta);
	if (DeltaLen > flow_constant(1e-6)) {
		Delta = Flow_V3_Mul_S(Delta, flow_constant(1.0) / DeltaLen);
		Body->Orientation = Flow_Quat_Mul_Quat(Flow_Quat_Axis_Angle(Delta, DeltaLen),
														 Body->Orientation);
		Body->Orientation = Flow_Quat_Norm(Body->Orientation);
		Flow_Assert(!Flow_Quat_Is_Nan(Body->Orientation));
	}
}

static void Flow_Body_Add_Position_Step(flow_body* Body, flow_v3 Delta) {
	Body->Position = Flow_V3_Add_V3(Body->Position, Delta);
	Flow_Assert(!Flow_V3_Is_Nan(Body->Position));
}

static void Flow_Body_Sub_Rotation_Step(flow_body* Body, flow_v3 Delta) {
	flow_real DeltaLen = Flow_V3_Mag(Delta);
	if (DeltaLen > flow_constant(1e-6)) {
		Delta = Flow_V3_Mul_S(Delta, flow_constant(1.0) / DeltaLen);
		Body->Orientation = Flow_Quat_Mul_Quat(Flow_Quat_Axis_Angle(Delta, -DeltaLen),
											   Body->Orientation);
		Body->Orientation = Flow_Quat_Norm(Body->Orientation);
		Flow_Assert(!Flow_Quat_Is_Nan(Body->Orientation));
	}
}

static void Flow_Body_Sub_Position_Step(flow_body* Body, flow_v3 Delta) {
	Body->Position = Flow_V3_Sub_V3(Body->Position, Delta);
	Flow_Assert(!Flow_V3_Is_Nan(Body->Position));
}

static void Flow_Add_Penetration(flow_system* System, flow_body* BodyA, flow_body* BodyB, flow_v3 P1, flow_v3 P2, flow_v3 V) {
	flow_penetration_array* Array = &System->PenetrationArray;
	if (Array->Count == Array->Capacity) {
		size_t NewCapacity = Flow_Max(Array->Capacity*2, 32);
		flow_penetration* NewPtr = (flow_penetration*)Flow_Allocate_Memory(&System->Allocator, NewCapacity * sizeof(flow_penetration));
		
		if (Array->Ptr) {
			Flow_Memcpy(NewPtr, Array->Ptr, Array->Capacity * sizeof(flow_penetration));
			Flow_Free_Memory(&System->Allocator, Array->Ptr);
		}

		Array->Ptr = NewPtr;
		Array->Capacity = NewCapacity;
	}

	flow_penetration* Penetration = Array->Ptr + Array->Count;
	Penetration->BodyA = Flow_Body_Pool_Get_ID(&System->BodyPool, BodyA);
	Penetration->BodyB = Flow_Body_Pool_Get_ID(&System->BodyPool, BodyB);
	Penetration->P1 = P1;
	Penetration->P2 = P2;
	Penetration->V  = V;

	Array->Count++;

	Flow_Debug_Record_Penetration(System, BodyA, BodyB, 
								  Penetration->P1, Penetration->P2, Penetration->V);
}

#define FLOW_COLLISION_QUERY_DEFINE(name) void name(flow_system* System, flow_body* BodyA, flow_m4* TransformA, flow_v3 ScaleA, flow_body* BodyB, flow_m4* TransformB, flow_v3 ScaleB)
typedef FLOW_COLLISION_QUERY_DEFINE(flow_collision_query_func);

static FLOW_COLLISION_QUERY_DEFINE(Flow_Sphere_Collisions) {
	Flow_Not_Implemented;
}

static FLOW_COLLISION_QUERY_DEFINE(Flow_Sphere_Box_Collisions) {
	Flow_Assert(BodyA->Collider.Type == FLOW_COLLIDER_TYPE_SPHERE &&
				BodyB->Collider.Type == FLOW_COLLIDER_TYPE_BOX);

	flow_sphere* Sphere = &BodyA->Collider.Sphere;
	flow_box* Box = &BodyB->Collider.Box;

	flow_real Radius = Flow_V3_Largest(ScaleA)*Sphere->Radius;
	flow_v3 HalfExtent = Flow_V3_Mul_V3(ScaleB, Box->HalfExtent);

	//Transform the box into the coordinate space of the sphere
	//Since the sphere doesn't change under orientation changes, we
	//don't need to take into account the orientation
	TransformB->t = Flow_V3_Sub_V3(TransformB->t, TransformA->t);

	flow_gjk_support SphereOriginSupportA = Flow_GJK_Origin();
	flow_gjk_support BoxSupportB = Flow_GJK_Extent(&System->TempArena, HalfExtent);
	flow_gjk_support BoxTransformSupportB = Flow_GJK_Transform(&System->TempArena, BoxSupportB, TransformB);

	flow_gjk_simplex Simplex;

	flow_real Distance;
	flow_penetration_test PenetrationTest = Flow_GJK_Penetration_Test(&SphereOriginSupportA, Radius, &BoxTransformSupportB, 0.0f, FLOW_COLLISION_TOLERANCE, &Simplex, &Distance);
	if (PenetrationTest.Intersected) {
		if (Distance == 0.0f) {
			flow_gjk_support RadiusSupportA = Flow_GJK_Radius(&System->TempArena, Radius);
			PenetrationTest = Flow_EPA_Penetration_Test_With_Simplex(&RadiusSupportA, &BoxTransformSupportB, FLOW_PENETRATION_TOLERANCE, &Simplex);
		}

		if (PenetrationTest.Intersected) {
			flow_v3 P1 = Flow_V3_Add_V3(PenetrationTest.P1, TransformA->t);
			flow_v3 P2 = Flow_V3_Add_V3(PenetrationTest.P2, TransformA->t);
			flow_v3 V  = PenetrationTest.V;

			Flow_Add_Penetration(System, BodyA, BodyB, P1, P2, V);
		}
	}
}

static FLOW_COLLISION_QUERY_DEFINE(Flow_Box_Sphere_Collisions) {
	Flow_Sphere_Box_Collisions(System, BodyB, TransformB, ScaleB, BodyA, TransformA, ScaleA);
}

static FLOW_COLLISION_QUERY_DEFINE(Flow_Box_Collisions) {
	Flow_Assert(BodyA->Collider.Type == FLOW_COLLIDER_TYPE_BOX &&
				BodyB->Collider.Type == FLOW_COLLIDER_TYPE_BOX);
	
	flow_box* BoxA = &BodyA->Collider.Box;
	flow_box* BoxB = &BodyB->Collider.Box;

	flow_v3 HalfExtentA = Flow_V3_Mul_V3(BoxA->HalfExtent, ScaleA);
	flow_v3 HalfExtentB = Flow_V3_Mul_V3(BoxB->HalfExtent, ScaleB);

	flow_gjk_support BoxSupportA = Flow_GJK_Extent(&System->TempArena, HalfExtentA);
	flow_gjk_support BoxSupportB = Flow_GJK_Extent(&System->TempArena, HalfExtentB);

	flow_m4 InvTransformA = Flow_M4_Inverse(TransformA);
	flow_m4 TransformBToA = Flow_M4_Mul_M4(TransformB, &InvTransformA);
	flow_gjk_support BoxTransformSupportB = Flow_GJK_Transform(&System->TempArena, BoxSupportB, &TransformBToA);

	flow_penetration_test PenetrationTest = Flow_EPA_Penetration_Test(&BoxSupportA, &BoxTransformSupportB, FLOW_COLLISION_TOLERANCE, FLOW_PENETRATION_TOLERANCE);
	if (PenetrationTest.Intersected) {
		flow_m3 TransformAM3 = Flow_M3_From_M4(TransformA);

		flow_v3 V = Flow_V3_Mul_M3(PenetrationTest.V, &TransformAM3);
		flow_v3 P1 = Flow_V4_Mul_M4(Flow_V4_From_V3(PenetrationTest.P1, flow_constant(1.0)), TransformA).xyz;
		flow_v3 P2 = Flow_V4_Mul_M4(Flow_V4_From_V3(PenetrationTest.P2, flow_constant(1.0)), TransformA).xyz;

		Flow_Add_Penetration(System, BodyA, BodyB, P1, P2, V);
	}
}

static flow_collision_query_func* G_CollisionQueryTable[FLOW_COLLIDER_TYPE_COUNT][FLOW_COLLIDER_TYPE_COUNT] = {
	{ Flow_Sphere_Collisions, Flow_Sphere_Box_Collisions },
	{ Flow_Box_Sphere_Collisions, Flow_Box_Collisions }
};

static flow_collision_query_func* Flow_Get_Collision_Query(flow_collider_type TypeA, flow_collider_type TypeB) {
	Flow_Assert(TypeA < FLOW_COLLIDER_TYPE_COUNT && TypeB < FLOW_COLLIDER_TYPE_COUNT);
	flow_collision_query_func* Result = G_CollisionQueryTable[TypeA][TypeB];
	return Result;
}

#define FLOW_CONTACTS_MIN_DISTANCE flow_constant(1.0e-6)
static flow_collision_point_array Flow_Prune_Contacts(flow_collision_point_array* Contacts, flow_v3 Normal) {
	Flow_Assert(Contacts->Count > 4);

	//Entire prunning process revolves around using the distance to the center of mass plus the penetration
	//depth to determine which points to keep. We never want a 0 value so we clamp against CONTACTS_MIN_DISTANCE
	flow_v3 Projected[FLOW_MAX_COLLISION_POINT_COUNT];
	flow_real PenetrationDepthSq[FLOW_MAX_COLLISION_POINT_COUNT];

	for (flow_u32 i = 0; i < Contacts->Count; i++) {
		//Project the contact points on the plane with a point through the center of mass of the contact
		//with the penetration normal. Since all points are already relative to the center of mass of the 
		//body1 we can just project these points onto the origin
		flow_v3 P1 = Contacts->Points[i].P1;
		Projected[i] = Flow_V3_Sub_V3(P1, Flow_V3_Mul_S(Normal, Flow_V3_Dot(P1, Normal)));

		//And get the penetration depth of each point
		PenetrationDepthSq[i] = Flow_Max(FLOW_CONTACTS_MIN_DISTANCE, Flow_V3_Sq_Mag(Flow_V3_Sub_V3(Contacts->Points[i].P2, P1)));
	}

	//Find the point that is furthest away from the center of mass as it has the largest torque influence
	//and the point that has the deepest penetration depth. Heuristic is distance*penetration_depth
	flow_u32 i1 = 0;
	flow_real MaxValue = Flow_Max(FLOW_CONTACTS_MIN_DISTANCE, Flow_V3_Sq_Mag(Projected[0]))*PenetrationDepthSq[0];
	for (flow_u32 i = 1; i < Contacts->Count; i++) {
		flow_real Value = Flow_Max(FLOW_CONTACTS_MIN_DISTANCE, Flow_V3_Sq_Mag(Projected[i]))*PenetrationDepthSq[i];
		if (Value > MaxValue) {
			MaxValue = Value;
			i1 = i;
		}
	}
	flow_v3 p1 = Projected[i1];

	//Find the furthest point from the first point and form a line segment
	flow_u32 i2 = (flow_u32)-1;
	MaxValue = -FLOW_MAX_REAL;
	for (flow_u32 i = 0; i < Contacts->Count; i++) {
		if (i != i1) {
			flow_real Value = Flow_Max(FLOW_CONTACTS_MIN_DISTANCE, Flow_V3_Sq_Mag(Projected[i]))*PenetrationDepthSq[i];
			if (Value > MaxValue) {
				MaxValue = Value;
				i2 = i;
			}
		}
	}

	Flow_Assert(i2 != (flow_u32)-1);
	flow_v3 p2 = Projected[i2];

	//Find the furthest points on both sides of the line segment to get the maximum area
	//on the contact manifold
	flow_u32 i3 = (flow_u32)-1;
	flow_u32 i4 = (flow_u32)-1;
	flow_real MinValue = 0.0f;
	MaxValue = 0.0f;
	flow_v3 Perp = Flow_V3_Cross(Flow_V3_Sub_V3(p2, p1), Normal);
	for (flow_u32 i = 0; i < Contacts->Count; i++) {
		if (i != i1 && i != i2) {
			flow_real Value = Flow_V3_Dot(Perp, Flow_V3_Sub_V3(Projected[i], p1));
			if (Value < MinValue) {
				MinValue = Value;
				i3 = i;
			} else if (Value > MaxValue) {
				MaxValue = Value;
				i4 = i;
			}
		}
	}

	//Add to the final result set while forming a polygon
	flow_collision_point_array Result = { 0 };
	Result.Points[Result.Count++] = Contacts->Points[i1];
	if (i3 != (flow_u32)-1) {
		Result.Points[Result.Count++] = Contacts->Points[i3];
	}

	Result.Points[Result.Count++] = Contacts->Points[i2];
	if (i4 != (flow_u32)-1) {
		Flow_Assert(i4 != i3);
		Result.Points[Result.Count++] = Contacts->Points[i4];
	}

	
	return Result;
}

typedef struct {
	flow_u32 Count;
	flow_v3  V[FLOW_MAX_COLLISION_POINT_COUNT];
} flow_support_face;

static void Flow_Transform_Face(flow_support_face* Face, const flow_m4* Transform) {
	for (flow_u32 i = 0; i < Face->Count; i++) {
		Face->V[i] = Flow_V4_Mul_M4(Flow_V4_From_V3(Face->V[i], flow_constant(1.0)), Transform).xyz;
	}
}

static flow_support_face Flow_Clip_Polygon_And_Plane(flow_support_face* PolygonToClip, flow_v3 PlaneOrigin, flow_v3 PlaneNormal) {
    Flow_Assert(PolygonToClip->Count >= 3);
    
	flow_support_face Result = { 0 };

	//Get the state of the last point
    flow_v3 e1 = PolygonToClip->V[PolygonToClip->Count-1];
    flow_real PrevNum = Flow_V3_Dot(Flow_V3_Sub_V3(PlaneOrigin, e1), PlaneNormal);
    flow_bool PrevInside = PrevNum < flow_constant(0.0);

    for(flow_u32 i = 0; i < PolygonToClip->Count; i++) {
        
		//Check if the next point is inside
		flow_v3 e2 = PolygonToClip->V[i];
        flow_real Num = Flow_V3_Dot(Flow_V3_Sub_V3(PlaneOrigin, e2), PlaneNormal);
        flow_bool CurInside = Num < flow_constant(0.0);

		//If we were previously inside and now outside (or vice versa) we add a point
		//on the clipping plane
        if(CurInside != PrevInside) {
            flow_v3 e12 = Flow_V3_Sub_V3(e2, e1);
            flow_real Denom = Flow_V3_Dot(e12, PlaneNormal);
            if(Denom != flow_constant(0.0)) {
				Result.V[Result.Count++] = Flow_V3_Add_V3(e1, Flow_V3_Mul_S(e12, PrevNum / Denom));
            } else {
                CurInside = PrevInside;
            }
        }

		//If we are inside, add it to the clipping plane
        if(CurInside) {
			Result.V[Result.Count++] = e2;
        }

		//Update the last state
        PrevNum = Num;
        PrevInside = CurInside;
        e1 = e2;
    }
	Flow_Assert(Result.Count < FLOW_MAX_COLLISION_POINT_COUNT);

    return Result;
}

static flow_support_face Flow_Clip_Polygons(flow_support_face* PolygonToClip, flow_support_face* ClippingPolygon, flow_v3 ClippingNormal) {
	flow_support_face Result = { 0 };

    Flow_Assert(PolygonToClip->Count >= 3 && ClippingPolygon->Count >= 3);

	flow_support_face TmpVertices[2] = { 0 };
    flow_u32 TmpVerticesIndex = 0;

    for(flow_u32 i = 0; i < ClippingPolygon->Count; i++) {
        //Compute the edge to clip against
        flow_v3 V1 = ClippingPolygon->V[i];
        flow_v3 V2 = ClippingPolygon->V[(i+1) % ClippingPolygon->Count];
		flow_v3 ClipNormal = Flow_V3_Cross(ClippingNormal, Flow_V3_Sub_V3(V2, V1));

		//Get the src and target polygons
		flow_support_face SrcPolygon = (i == 0) ? *PolygonToClip : TmpVertices[TmpVerticesIndex];
        TmpVerticesIndex ^= 1;

        flow_support_face* TargetPolygons = (i == ClippingPolygon->Count - 1) ? &Result : &TmpVertices[TmpVerticesIndex];
		TargetPolygons->Count = 0;

		//Clip the source polygon against the clipping plane (origin plus normal)
        *TargetPolygons = Flow_Clip_Polygon_And_Plane(&SrcPolygon, V1, ClipNormal);

		//If we don't have any polygons left, break
        if(TargetPolygons->Count < 3) {
			Result.Count = 0;
            break;
        }
    }

    return Result;
}

static flow_collision_point_array Flow_Build_Manifold(flow_v3 Direction, flow_support_face* FaceA, flow_support_face* FaceB, flow_real SpeculativeContactDistSq) {
	flow_collision_point_array Result = { 0 };
	
	//Check if we have valid shapes
	if (FaceA->Count >= 2 && FaceB->Count >= 2) {
		//Clip FaceB against FaceA
		
		flow_support_face ClippedFace = { 0 };
		if (FaceA->Count >= 3 && FaceB->Count >= 3) {
			ClippedFace = Flow_Clip_Polygons(FaceB, FaceA, Direction);
		} else {
			//Need to implement polygon/edge clipping and
			//edge/edge clipping
			Flow_Not_Implemented;
		}

		//Find the plane normal. We need to project points onto the plane of FaceA and keep points
		//behind that plane only

		flow_v3 PlaneOrigin = FaceA->V[0];
		flow_v3 FirstEdge = Flow_V3_Sub_V3(FaceA->V[1], PlaneOrigin);

		flow_v3 PlaneNormal;
		if (FaceA->Count >= 3) {
			//Normal is just the plane normal
			PlaneNormal = Flow_V3_Cross(FirstEdge, Flow_V3_Sub_V3(FaceA->V[2], PlaneOrigin));
		} else {
			//If FaceA has two vertices only. To calculate a normal find a vector perpendicular to the edge
			//and penetration axis and then use that vector to find the normal
			PlaneNormal = Flow_V3_Cross(Flow_V3_Cross(FirstEdge, Direction), FirstEdge);
		}

		//If the normal is 0, the clipped shape won't yield contact points
		flow_real PlaneNormalSqLen = Flow_V3_Sq_Mag(PlaneNormal);
		if (PlaneNormalSqLen > flow_constant(0.0)) {
			//Add points that are behind the plane and within a certain distance

			for (flow_u32 i = 0; i < ClippedFace.Count; i++) {
				flow_v3 P2 = ClippedFace.V[i];

				//Distance should be divided by the length of the plane normal, but we take this into account
				//layer
				flow_real Distance = Flow_V3_Dot(Flow_V3_Sub_V3(P2, PlaneOrigin), PlaneNormal);
				
				//Must be close enough to the plane. We divide by the length of the normal here
				if (Distance <= 0.0f || Flow_Sq(Distance) < SpeculativeContactDistSq * PlaneNormalSqLen) {
					//Project P2 back on the first shape using the normal to get the other contact point
					flow_v3 P1 = Flow_V3_Sub_V3(P2, Flow_V3_Mul_S(PlaneNormal, Distance / PlaneNormalSqLen));

					Result.Points[Result.Count].P1 = P1;
					Result.Points[Result.Count].P2 = P2;
					Result.Count++;
				}
			}
		}
	}
	Flow_Assert(Result.Count < FLOW_MAX_COLLISION_POINT_COUNT);

	return Result;
}

static flow_support_face Flow_Get_Box_Support_Face(flow_v3 HalfExtent, flow_v3 Direction) {
	flow_support_face Result = { 4 };

	size_t Axis = Flow_V3_Largest_Index(Direction);
	
	if (Direction.Data[Axis] >= 0) {
		switch(Axis) {
            case 0: {
                Result.V[0] = Flow_V3(HalfExtent.x, -HalfExtent.y, -HalfExtent.z);
                Result.V[1] = Flow_V3(HalfExtent.x,  HalfExtent.y, -HalfExtent.z);
                Result.V[2] = Flow_V3(HalfExtent.x,  HalfExtent.y,  HalfExtent.z);
                Result.V[3] = Flow_V3(HalfExtent.x, -HalfExtent.y,  HalfExtent.z);
            } break;

            case 1: {
                Result.V[0] = Flow_V3(-HalfExtent.x, HalfExtent.y, -HalfExtent.z);
                Result.V[1] = Flow_V3(-HalfExtent.x, HalfExtent.y,  HalfExtent.z);
                Result.V[2] = Flow_V3( HalfExtent.x, HalfExtent.y,  HalfExtent.z);
                Result.V[3] = Flow_V3( HalfExtent.x, HalfExtent.y, -HalfExtent.z);
            } break;

            case 2: {
                Result.V[0] = Flow_V3(-HalfExtent.x, -HalfExtent.y, HalfExtent.z);
                Result.V[1] = Flow_V3( HalfExtent.x, -HalfExtent.y, HalfExtent.z);
                Result.V[2] = Flow_V3( HalfExtent.x,  HalfExtent.y, HalfExtent.z);
                Result.V[3] = Flow_V3(-HalfExtent.x,  HalfExtent.y, HalfExtent.z);
            } break;

            Flow_Invalid_Default_Case;
        }
	} else {
		switch(Axis) {
            case 0: {
                Result.V[0] = Flow_V3(-HalfExtent.x, -HalfExtent.y, -HalfExtent.z);
                Result.V[1] = Flow_V3(-HalfExtent.x, -HalfExtent.y,  HalfExtent.z);
                Result.V[2] = Flow_V3(-HalfExtent.x,  HalfExtent.y,  HalfExtent.z);
                Result.V[3] = Flow_V3(-HalfExtent.x,  HalfExtent.y, -HalfExtent.z);
            } break;

            case 1: {
                Result.V[0] = Flow_V3(-HalfExtent.x, -HalfExtent.y, -HalfExtent.z);
                Result.V[1] = Flow_V3( HalfExtent.x, -HalfExtent.y, -HalfExtent.z);
                Result.V[2] = Flow_V3( HalfExtent.x, -HalfExtent.y,  HalfExtent.z);
                Result.V[3] = Flow_V3(-HalfExtent.x, -HalfExtent.y,  HalfExtent.z);
            } break;

            case 2: {
                Result.V[0] = Flow_V3(-HalfExtent.x, -HalfExtent.y, -HalfExtent.z);
                Result.V[1] = Flow_V3(-HalfExtent.x,  HalfExtent.y, -HalfExtent.z);
                Result.V[2] = Flow_V3( HalfExtent.x,  HalfExtent.y, -HalfExtent.z);
                Result.V[3] = Flow_V3( HalfExtent.x, -HalfExtent.y, -HalfExtent.z);
            } break;

            Flow_Invalid_Default_Case;
        }
	}

	return Result;
}

static flow_support_face Flow_Body_Get_Support_Face(flow_body* Body, flow_v3 Direction) {
	flow_support_face SupportFace = { 0 };
	switch (Body->Collider.Type) {
		case FLOW_COLLIDER_TYPE_SPHERE: {
		} break;

		case FLOW_COLLIDER_TYPE_BOX: {
			flow_v3 Extent = Flow_V3_Mul_V3(Body->Collider.Box.HalfExtent, Body->Scale);
			SupportFace = Flow_Get_Box_Support_Face(Extent, Direction);
		} break;

		Flow_Invalid_Default_Case;
	}
	return SupportFace;
}

typedef struct {
	flow_real Mass;
	flow_m3 Inertia;
} flow_mass;

static flow_real Flow_Safe_Inv_Mass(flow_real Mass) {
    flow_real Result = Flow_Equal_Zero_Eps(Mass) ? flow_constant(0.0) : flow_constant(1.0)/Mass;
	return Result;
}

static flow_mass Flow_Get_Sphere_Mass(flow_sphere* Sphere) {
	flow_mass Result = { 0 };
	
	flow_real Radius = Sphere->Radius;
	flow_real SqRadius = Flow_Sq(Radius);

	Result.Mass = Sphere->Density*flow_constant(1.333)*FLOW_PI*SqRadius*Radius;

	flow_real InertiaScalar = 0.4f * Result.Mass*SqRadius;
	Result.Inertia = Flow_M3_Scale_All(InertiaScalar);
	
	return Result;
}

static flow_mass Flow_Get_Box_Mass(flow_box* Box) {
	flow_v3 Extent = Flow_V3_Mul_S(Box->HalfExtent, 2.0f);
	flow_real Mass = Extent.x * Extent.y * Extent.z * Box->Density;
	flow_v3 SizeSq = Flow_V3_Mul_V3(Extent, Extent);
	flow_v3 Scale = Flow_V3_Mul_S(Flow_V3_Add_V3(Flow_V3(SizeSq.y, SizeSq.x, SizeSq.x), 
												 Flow_V3(SizeSq.z, SizeSq.z, SizeSq.y)),
						Mass / flow_constant(12.0));

	flow_mass Result = { Mass, Flow_M3_Scale(Scale) };
	return Result;

}

static flow_mass Flow_Scale_Mass(flow_mass Mass, flow_v3 s) {
	flow_mass Result = { 0 };

	flow_real ScaleFactor = s.x*s.y*s.z;
	Result.Mass = Mass.Mass*ScaleFactor;

	flow_real sxy = s.x*s.y;
	flow_real sxz = s.x*s.z;
	flow_real syz = s.y*s.z;

	flow_m3* I = &Mass.Inertia;

	flow_m3 Inertia = {
		Flow_Sq(s.x)*I->m00, sxy*I->m01, sxz*I->m02,
		sxy*I->m10, Flow_Sq(s.y)*I->m11, syz*I->m12,
		sxz*I->m20, syz*I->m21, Flow_Sq(s.z)*I->m22
	};

	Result.Inertia = Flow_M3_Mul_S(&Inertia, ScaleFactor);
	return Result;
}

static flow_mass Flow_Body_Get_Mass(flow_body* Body) {
	flow_mass Mass = { 0 };
	if (Body->Type != FLOW_BODY_STATIC) {
		switch (Body->Collider.Type) {
			case FLOW_COLLIDER_TYPE_SPHERE: {
				flow_sphere Sphere = Body->Collider.Sphere;
				Sphere.Radius = Sphere.Radius * Flow_V3_Largest(Body->Scale);
				Mass = Flow_Get_Sphere_Mass(&Sphere);
			} break;

			case FLOW_COLLIDER_TYPE_BOX: {
				flow_box Box = Body->Collider.Box;
				Box.HalfExtent = Flow_V3_Mul_V3(Box.HalfExtent, Body->Scale);
				Mass = Flow_Get_Box_Mass(&Box);
			} break;

			Flow_Invalid_Default_Case;
		}
	}

	return Mass;
}

static flow_v3 Flow_Collider_Get_COM(flow_collider* Collider) {
	//Right now we only have spheres and boxes which have center of masses around the origin
	flow_v3 Result = Flow_V3_Zero();
	return Result;
}

static flow_v3 Flow_Body_Get_COM(flow_body* Body) {
	flow_v3 Result = Flow_Collider_Get_COM(&Body->Collider);
	flow_m4 Transform = Flow_M4_Transform(Body->Position, Body->Orientation);
	Result = Flow_V4_Mul_M4(Flow_V4_From_V3(Result, flow_constant(1.0f)), &Transform).xyz;
	return Result;
}

static flow_m3 Flow_Body_Get_World_Inertia(flow_body* Body, flow_m3* Inertia) {
	flow_m3 Orientation = Flow_M3_From_Quat(Body->Orientation);
	flow_m3 OrientationT = Flow_M3_Transpose(&Orientation);
	flow_m3 LocalInertiaTemp = Flow_M3_Mul_M3(&OrientationT, Inertia);
	return Flow_M3_Mul_M3(&LocalInertiaTemp, &Orientation);
}

static void 
Flow_Contact_Constraint_Init_With_Bias(flow_contact_constraint* Constraint, 
									   flow_v3 R1, flow_v3 R2, flow_real InvMass1, flow_real InvMass2, 
									   flow_m3* InvInertia1, flow_m3* InvInertia2, flow_v3 Normal, flow_real Bias) {
	//Get the properties used for solving contact constraints
	Constraint->R1Axis = Flow_V3_Cross(R1, Normal);
	Constraint->R2Axis = Flow_V3_Cross(R2, Normal);

	//So we don't have to store the inertia tensors in the constraint, precompute the 
	//inertia tensor multiplication on the R axes.
	Constraint->InvInertiaR1Axis = Flow_V3_Mul_M3(Constraint->R1Axis, InvInertia1);
	Constraint->InvInertiaR2Axis = Flow_V3_Mul_M3(Constraint->R2Axis, InvInertia2);

	//Calculate the inverse effective mass: K = J * M^-1 * J^T
	flow_real InvEffectiveMass = InvMass1 + Flow_V3_Dot(Constraint->InvInertiaR1Axis, Constraint->R1Axis);
	InvEffectiveMass += InvMass2 + Flow_V3_Dot(Constraint->InvInertiaR2Axis, Constraint->R2Axis);
	Flow_Assert(!Flow_Equal_Zero_Eps(InvEffectiveMass));

	Constraint->EffectiveMass = flow_constant(1.0) / InvEffectiveMass;
	Constraint->Bias = Bias;
	Constraint->TotalLambda = flow_constant(0.0);
	Constraint->Active = flow_true;
}

static void Flow_Contact_Constraint_Init(flow_contact_constraint* Constraint, 
										 flow_v3 R1, flow_v3 R2, flow_real InvMass1, flow_real InvMass2,
										 flow_m3* InvInertia1, flow_m3* InvInertia2, flow_v3 Normal) {
	Flow_Contact_Constraint_Init_With_Bias(Constraint, R1, R2, InvMass1, InvMass2, 
										   InvInertia1, InvInertia2, Normal, flow_constant(0.0));
}

static flow_real Flow_Contact_Constraint_Get_Velocity_Lambda(flow_contact_constraint* Constraint, flow_body* Body1, flow_body* Body2, flow_v3 Normal) {
	Flow_Assert(Constraint->Active);
	flow_real jv = flow_constant(0.0);

	//Calculate jacobian for linear velocity
    jv = Flow_V3_Dot(Flow_V3_Sub_V3(Body1->LinearVelocity, Body2->LinearVelocity), Normal);
    
	//Calculate jacobian for angular velocity
	jv += Flow_V3_Dot(Body1->AngularVelocity, Constraint->R1Axis);
    jv -= Flow_V3_Dot(Body2->AngularVelocity, Constraint->R2Axis);

	//Lagrange multipler: -K^-1 (Jv + b)
    flow_real Lambda = Constraint->EffectiveMass * (jv - Constraint->Bias);
    return Constraint->TotalLambda+Lambda;
}

static flow_real Flow_Contact_Constraint_Update_Lambda_And_Get_Delta(flow_contact_constraint* Constraint, flow_real Lambda) {
	Flow_Assert(Constraint->Active);
    flow_real DeltaLambda = Lambda-Constraint->TotalLambda; //Get change in lambda
    Constraint->TotalLambda = Lambda; //Store the accumulated impulse
    return DeltaLambda;
}

static void Flow_Contact_Constraint_Apply_Velocity(flow_contact_constraint* Constraint, flow_body* Body1, flow_body* Body2, flow_real InvMass1, flow_real InvMass2, flow_v3 Normal, flow_real Lambda) { 
    Flow_Assert(Constraint->Active);
	
	//Calculate the velocity change from the constraint
	//Impulse (P): J^T * Lambda
	//New Velocity: v + M^-1*P
	
	Body1->LinearVelocity = Flow_V3_Sub_V3(Body1->LinearVelocity, Flow_V3_Mul_S(Normal, (Lambda*InvMass1)));
    Body1->AngularVelocity = Flow_V3_Sub_V3(Body1->AngularVelocity, Flow_V3_Mul_S(Constraint->InvInertiaR1Axis, Lambda));

	Body2->LinearVelocity = Flow_V3_Add_V3(Body2->LinearVelocity, Flow_V3_Mul_S(Normal, (Lambda*InvMass2)));
    Body2->AngularVelocity = Flow_V3_Add_V3(Body2->AngularVelocity, Flow_V3_Mul_S(Constraint->InvInertiaR2Axis, Lambda));
}

static void Flow_Contact_Constraint_Update_And_Apply_Velocity(flow_contact_constraint* Constraint, flow_body* Body1, flow_body* Body2, flow_real InvMass1, flow_real InvMass2, flow_v3 Normal, flow_real Lambda) {
    Flow_Assert(Constraint->Active);
	Lambda = Flow_Contact_Constraint_Update_Lambda_And_Get_Delta(Constraint, Lambda);
    Flow_Contact_Constraint_Apply_Velocity(Constraint, Body1, Body2, InvMass1, InvMass2, Normal, Lambda);
}

static void Flow_Contact_Constraint_Solve_Velocity(flow_contact_constraint* Constraint, flow_body* Body1, flow_body* Body2, 
												   flow_real InvMass1, flow_real InvMass2, flow_v3 Normal) {
	Flow_Assert(Constraint->Active);
	flow_real Lambda = Flow_Max(flow_constant(0.0), Flow_Contact_Constraint_Get_Velocity_Lambda(Constraint, Body1, Body2, Normal));
	Flow_Contact_Constraint_Update_And_Apply_Velocity(Constraint, Body1, Body2, InvMass1, InvMass2, 
													  Normal, Lambda);
}

static void Flow_Contact_Constraint_Apply_Warm_Start(flow_contact_constraint* Constraint, flow_body* Body1, flow_body* Body2, flow_real InvMass1, flow_real InvMass2, flow_v3 Normal) {
	Flow_Assert(Constraint->Active);
	Flow_Contact_Constraint_Apply_Velocity(Constraint, Body1, Body2, InvMass1, InvMass2, Normal, Constraint->TotalLambda);
}

static void Flow_Contact_Constraint_Solve_Position(flow_contact_constraint* Constraint, flow_body* Body1, flow_body* Body2,
												   flow_real InvMass1, flow_real InvMass2, flow_v3 Normal, flow_real Penetration,
												   flow_real Baumguarte) {
	Flow_Assert(Constraint->Active);
	if (Penetration != flow_constant(0.0)) {
		flow_real Lambda = -Constraint->EffectiveMass * Baumguarte * Penetration;

		Flow_Body_Sub_Position_Step(Body1, Flow_V3_Mul_S(Normal, Lambda * InvMass1));
		Flow_Body_Sub_Rotation_Step(Body1, Flow_V3_Mul_S(Constraint->InvInertiaR1Axis, Lambda));

		Flow_Body_Add_Position_Step(Body2, Flow_V3_Mul_S(Normal, Lambda * InvMass2));
		Flow_Body_Add_Rotation_Step(Body2, Flow_V3_Mul_S(Constraint->InvInertiaR2Axis, Lambda));
	}
}

static void Flow_Add_Collision_Constraint(flow_system* System, flow_body* Body1, flow_body* Body2,
										  flow_v3 V, flow_collision_point_array* Points) {

	flow_collision_constraint_array* Array = &System->CollisionConstraintArray;

	if (Array->Count == Array->Capacity) {
		size_t NewCapacity = Flow_Max(Array->Capacity*2, 32);
		flow_collision_constraint* NewPtr = (flow_collision_constraint*)Flow_Allocate_Memory(&System->Allocator, NewCapacity * sizeof(flow_collision_constraint));
		
		if (Array->Ptr) {
			Flow_Memcpy(NewPtr, Array->Ptr, Array->Capacity * sizeof(flow_collision_constraint));
			Flow_Free_Memory(&System->Allocator, Array->Ptr);
		}

		Array->Ptr = NewPtr;
		Array->Capacity = NewCapacity;
	}

	flow_m3 Basis = Flow_M3_Basis(V);
	flow_real CombinedFriction = Flow_Combine_Friction(Body1->Friction, Body2->Friction);

	flow_collision_constraint* Constraint = Array->Ptr + Array->Count++;
	Constraint->Normal = Basis.z;
	Constraint->Tangents[0] = Basis.x;
	Constraint->Tangents[1] = Basis.y;
	Constraint->Friction = CombinedFriction;

	flow_mass MassInfo1 = Flow_Body_Get_Mass(Body1);
	Constraint->Body1 = Body1;
	Constraint->InvMass1 = Flow_Safe_Inv_Mass(MassInfo1.Mass);
	flow_v3 CenterOfMass1 = Flow_Body_Get_COM(Body1);

	flow_mass MassInfo2 = Flow_Body_Get_Mass(Body2);
	Constraint->InvMass2 = Flow_Safe_Inv_Mass(MassInfo2.Mass);
	Constraint->Body2 = Body2;
	flow_v3 CenterOfMass2 = Flow_Body_Get_COM(Body2);

	flow_m3 InertiaWorld1 = Flow_Body_Get_World_Inertia(Body1, &MassInfo1.Inertia);
	flow_m3 InertiaWorld2 = Flow_Body_Get_World_Inertia(Body2, &MassInfo2.Inertia);

	flow_m3 InvInertiaWorld1 = Flow_M3_Inverse(&InertiaWorld1);
	flow_m3 InvInertiaWorld2 = Flow_M3_Inverse(&InertiaWorld2);

	flow_m4 InvTransform1 = Flow_M4_Inverse_Transform(Body1->Position, Body1->Orientation);
	flow_m4 InvTransform2 = Flow_M4_Inverse_Transform(Body2->Position, Body2->Orientation);

	Constraint->ContactCount = Points->Count;
	Flow_Assert(Constraint->ContactCount <= Flow_Array_Count(Constraint->Contacts));

	for (size_t i = 0; i < Points->Count; i++) {
		flow_contact* Contact = Constraint->Contacts + i;
		flow_collision_point* CollisionPoint = Points->Points + i;

		flow_v3 P = Flow_V3_Mul_S(Flow_V3_Add_V3(CollisionPoint->P1, CollisionPoint->P2), flow_constant(0.5));
		flow_v3 R1 = Flow_V3_Sub_V3(P, CenterOfMass1);
		flow_v3 R2 = Flow_V3_Sub_V3(P, CenterOfMass2);

		Contact->ContactPoint.LocalP1 = Flow_V4_Mul_M4(Flow_V4_From_V3(CollisionPoint->P1, flow_constant(1.0)), &InvTransform1).xyz;
		Contact->ContactPoint.LocalP2 = Flow_V4_Mul_M4(Flow_V4_From_V3(CollisionPoint->P2, flow_constant(1.0)), &InvTransform2).xyz;

		Flow_Contact_Constraint_Init(&Contact->NormalConstraint, R1, R2, Constraint->InvMass1, Constraint->InvMass2, &InvInertiaWorld1, &InvInertiaWorld2, Constraint->Normal);
		
		if (CombinedFriction > 0) {
			Flow_Contact_Constraint_Init(&Contact->TangentConstraints[0], R1, R2, Constraint->InvMass1, Constraint->InvMass2, &InvInertiaWorld1, &InvInertiaWorld2, Constraint->Tangents[0]);
			Flow_Contact_Constraint_Init(&Contact->TangentConstraints[1], R1, R2, Constraint->InvMass1, Constraint->InvMass2, &InvInertiaWorld1, &InvInertiaWorld2, Constraint->Tangents[1]);
		}
	}
}

typedef struct flow_body_pair flow_body_pair;
struct flow_body_pair {
	flow_u64 		Hash;
	flow_u32 		IndexA;
	flow_u32 		IndexB;
	flow_body_pair* Next;
};

typedef struct {
	flow_body_pair* First;
	flow_body_pair* Last;
} flow_body_pair_slot;

FLOWDEF flow_system* Flow_System_Create(const flow_system_create_info* CreateInfo) {
	flow_allocator Allocator = {};
	if (CreateInfo->Allocator) {
		Allocator = *CreateInfo->Allocator;
	} else {
#ifdef FLOW_NO_STDLIB
		return NULL;
#else
		Allocator.AllocateMemory = Flow_Malloc;
		Allocator.FreeMemory = Flow_Free;
#endif
	}

	flow_system* Result = (flow_system*)Flow_Allocate_Memory(&Allocator, sizeof(flow_system));
	Result->Allocator = Allocator;

	Flow_Arena_Init(&Result->TempArena, Result->Allocator);

	Flow_Body_Pool_Init(&Result->BodyPool, Result->Allocator);

	Result->Gravity = Flow_V3(0.0f, 0.0f, -9.8f);

	return Result;
}

FLOWDEF void Flow_System_Delete(flow_system* System) {
	flow_allocator* Allocator = &System->Allocator;

	//This must be last
	Flow_Free_Memory(Allocator, System);
}

FLOWDEF void Flow_System_Update(flow_system* System, flow_f64 dt) {
	Flow_Debug_Clear(System);
	
	System->dt = dt;

	flow_body_pool* BodyPool = &System->BodyPool;
	Flow_Arena_Clear(&System->TempArena);

	//Integrate
	for (size_t i = 0; i < BodyPool->MaxUsed; i++) {
		if (BodyPool->IDs[i].Index == i) {
			flow_body* Body = BodyPool->Bodies + i;
			if (Body->Type == FLOW_BODY_DYNAMIC) {
				flow_mass MassInfo = Flow_Body_Get_Mass(Body);
				flow_real InvMass = Flow_Safe_Inv_Mass(MassInfo.Mass);

				flow_m3 WorldInertiaTensor = Flow_Body_Get_World_Inertia(Body, &MassInfo.Inertia);
				flow_m3 WorldInvInertiaTensor = Flow_M3_Inverse(&WorldInertiaTensor);

				flow_v3 LinearAcceleration = Flow_V3_Add_V3(System->Gravity, Flow_V3_Mul_S(Body->Force, InvMass));
				flow_v3 AngularAcceleration = Flow_V3_Mul_M3(Body->Torque, &WorldInvInertiaTensor);

				Body->LinearVelocity = Flow_V3_Add_V3(Body->LinearVelocity, Flow_V3_Mul_S(LinearAcceleration, flow_constant(System->dt)));
				Body->AngularVelocity = Flow_V3_Add_V3(Body->AngularVelocity, Flow_V3_Mul_S(AngularAcceleration, flow_constant(System->dt)));
			
				Body->Force = Flow_V3_Zero();
				Body->Torque = Flow_V3_Zero();
			}
		}
	}

	//Collect pairs
	flow_u32 SlotCapacity = Flow_Ceil_Pow2_U32(BodyPool->Count*BodyPool->Count);
	flow_u32 SlotMask = SlotCapacity - 1;
	flow_body_pair_slot* Slots = Flow_Arena_Push_Array(&System->TempArena, SlotCapacity, flow_body_pair_slot);

	for (flow_u32 i = 0; i < BodyPool->MaxUsed; i++) {
		if (BodyPool->IDs[i].Index == i) {
			if (BodyPool->Bodies[i].Type == FLOW_BODY_DYNAMIC) {
				for (flow_u32 j = 0; j < BodyPool->MaxUsed; j++) {
					if (BodyPool->IDs[j].Index == j) {
						if (i != j) {

							flow_u32 IndexA = i;
							flow_u32 IndexB = j;
							if (IndexA > IndexB) {
								flow_u32 Tmp = IndexA;
								IndexA = IndexB;
								IndexB = Tmp;
							}

							flow_u64 Hash = Flow_Hash_U64(IndexA | ((flow_u64)IndexB << 32));
							flow_u64 SlotIndex = Hash & SlotMask;
							flow_body_pair_slot* Slot = Slots + SlotIndex;

							flow_body_pair* Pair = NULL;
							for (flow_body_pair* HashPair = Slot->First; HashPair; HashPair = HashPair->Next) {
								if (HashPair->Hash == Hash && HashPair->IndexA == IndexA && HashPair->IndexB == IndexB) {
									Pair = HashPair;
									break;
								}
							}

							if (!Pair) {
								flow_body_pair* Pair = Flow_Arena_Push_Struct(&System->TempArena, flow_body_pair);
								Pair->Hash = Hash;
								Pair->IndexA = IndexA;
								Pair->IndexB = IndexB;
								Flow_SLL_Push_Back(Slot->First, Slot->Last, Pair);
							}
						}
					}
				}
			}
		}
	}

	//Collide pairs
	flow_penetration_array* PenetrationArray = &System->PenetrationArray;
	PenetrationArray->Count = 0;

	for (flow_u32 i = 0; i < SlotCapacity; i++) {
		flow_body_pair_slot* Slot = Slots + i;
		for (flow_body_pair* Pair = Slot->First; Pair; Pair = Pair->Next) {
			flow_body* BodyA = BodyPool->Bodies + Pair->IndexA;
			flow_body* BodyB = BodyPool->Bodies + Pair->IndexB;

			//Make sure the dynamic body is in BodyA
			if (BodyA->Type != FLOW_BODY_DYNAMIC) {
				Flow_Assert(BodyB->Type == FLOW_BODY_DYNAMIC);
				flow_body* Tmp = BodyA;
				BodyA = BodyB;
				BodyB = Tmp;
			}


			flow_collision_query_func* CollisionQuery = Flow_Get_Collision_Query(BodyA->Collider.Type, BodyB->Collider.Type);

			flow_m4 TransformA = Flow_M4_Transform(BodyA->Position, BodyA->Orientation);
			flow_m4 TransformB = Flow_M4_Transform(BodyB->Position, BodyB->Orientation);

			CollisionQuery(System, BodyA, &TransformA, BodyA->Scale, BodyB, &TransformB, BodyB->Scale);
		}
	}

	//Add collision constraints
	flow_collision_constraint_array* CollisionConstraintArray = &System->CollisionConstraintArray;
	CollisionConstraintArray->Count = 0;
	for (size_t i = 0; i < PenetrationArray->Count; i++) {
		flow_penetration* Penetration = PenetrationArray->Ptr + i;
		
		flow_body* BodyA = Flow_Get_Body(System, Penetration->BodyA);
		flow_body* BodyB = Flow_Get_Body(System, Penetration->BodyB);
		if (BodyA && BodyB) {
			Flow_Assert(BodyA->Type != FLOW_BODY_STATIC);

			flow_m4 TransformA = Flow_M4_Transform(BodyA->Position, BodyA->Orientation);
			flow_m4 TransformB = Flow_M4_Transform(BodyB->Position, BodyB->Orientation);

			flow_m3 ModelToWorldA = Flow_M3_From_Quat(BodyA->Orientation);
			flow_m3 WorldToModelA = Flow_M3_Transpose(&ModelToWorldA);
			flow_v3 ALocalV = Flow_V3_Mul_M3(Penetration->V, &WorldToModelA);

			flow_m3 ModelToWorldB = Flow_M3_From_Quat(BodyB->Orientation);
			flow_m3 WorldToModelB = Flow_M3_Transpose(&ModelToWorldB);
			flow_v3 BLocalV = Flow_V3_Mul_M3(Penetration->V, &WorldToModelB);

			flow_support_face FaceA = Flow_Body_Get_Support_Face(BodyA, ALocalV);
			flow_support_face FaceB = Flow_Body_Get_Support_Face(BodyB, Flow_V3_Negate(BLocalV));

			Flow_Transform_Face(&FaceA, &TransformA);
			Flow_Transform_Face(&FaceB, &TransformB);

			flow_collision_point_array CollisionPoints = Flow_Build_Manifold(Penetration->V, &FaceA, &FaceB, FLOW_MANIFOLD_TOLERANCE_SQ);
			if (!CollisionPoints.Count) {
				CollisionPoints.Count++;
				CollisionPoints.Points[0].P1 = Penetration->P2;
				CollisionPoints.Points[0].P2 = Penetration->P1;
			}

			if (CollisionPoints.Count > 4) {
				//Need to prune contacts and limit it to 4
				CollisionPoints = Flow_Prune_Contacts(&CollisionPoints, Penetration->V);
			}

			Flow_Add_Collision_Constraint(System, BodyA, BodyB, Penetration->V, &CollisionPoints);
		}
	}

	//Solve velocity constraints
	for (size_t i = 0; i < FLOW_MAX_VELOCITY_ITERATIONS; i++) {
		for (size_t c = 0; c < CollisionConstraintArray->Count; c++) {
			flow_collision_constraint* Constraint = CollisionConstraintArray->Ptr + c;

			flow_body* Body1 = Constraint->Body1;
			flow_body* Body2 = Constraint->Body2;

			flow_real InvMass1 = Constraint->InvMass1;
			flow_real InvMass2 = Constraint->InvMass2;

			for (size_t j = 0; j < Constraint->ContactCount; j++) {
				flow_contact* Contact = Constraint->Contacts + j;
				if (Contact->TangentConstraints[0].Active) {
					Flow_Assert(Contact->TangentConstraints[1].Active);
					flow_real Lambda1 = Flow_Contact_Constraint_Get_Velocity_Lambda(&Contact->TangentConstraints[0], Body1, Body2, Constraint->Tangents[0]);
					flow_real Lambda2 = Flow_Contact_Constraint_Get_Velocity_Lambda(&Contact->TangentConstraints[1], Body1, Body2, Constraint->Tangents[1]);
					
					flow_real TotalLambdaSq = Flow_Sq(Lambda1) + Flow_Sq(Lambda2);
					flow_real MaxLambda = Constraint->Friction * Contact->NormalConstraint.TotalLambda;
					if (TotalLambdaSq > Flow_Sq(MaxLambda)) {
						flow_real Scale = MaxLambda / Flow_Sqrt(TotalLambdaSq);
						Lambda1 *= Scale;
						Lambda2 *= Scale;
					}

					Flow_Contact_Constraint_Update_And_Apply_Velocity(&Contact->TangentConstraints[0], Body1, Body2, 
																	  InvMass1, InvMass2, Constraint->Tangents[0], Lambda1);

					Flow_Contact_Constraint_Update_And_Apply_Velocity(&Contact->TangentConstraints[1], Body1, Body2, 
																	  InvMass1, InvMass2, Constraint->Tangents[1], Lambda2);
				}

				Flow_Contact_Constraint_Solve_Velocity(&Contact->NormalConstraint, Body1, Body2, InvMass1, InvMass2, Constraint->Normal);
			}
		}
	}

	//Apply positional updates after constraints are solved
	for (size_t i = 0; i < BodyPool->MaxUsed; i++) {
		if (BodyPool->IDs[i].Index == i) {
			flow_body* Body = BodyPool->Bodies + i;
			if (Body->Type == FLOW_BODY_DYNAMIC) {
				Flow_Body_Add_Position_Step(Body, Flow_V3_Mul_S(Body->LinearVelocity, flow_constant(System->dt)));
				Flow_Body_Add_Rotation_Step(Body, Flow_V3_Mul_S(Body->AngularVelocity, flow_constant(System->dt)));
			}
		}
	}

	//Solve position constraints
	#pragma warning(disable : 4296)
	for (size_t i = 0; i < FLOW_MAX_POSITION_ITERATIONS; i++) {
		for (size_t c = 0; c < CollisionConstraintArray->Count; c++) {
			flow_collision_constraint* Constraint = CollisionConstraintArray->Ptr + c;

			flow_body* Body1 = Constraint->Body1;
			flow_body* Body2 = Constraint->Body2;

			flow_mass Mass1 = Flow_Body_Get_Mass(Body1);
			flow_mass Mass2 = Flow_Body_Get_Mass(Body2);

			flow_real InvMass1 = Flow_Safe_Inv_Mass(Mass1.Mass);
			flow_real InvMass2 = Flow_Safe_Inv_Mass(Mass2.Mass);

			flow_m3 InvInertia1 = Flow_M3_Inverse(&Mass1.Inertia);
			flow_m3 InvInertia2 = Flow_M3_Inverse(&Mass2.Inertia);

			for (size_t j = 0; j < Constraint->ContactCount; j++) {
				flow_contact* Contact = Constraint->Contacts + j;

				flow_m4 Transform1 = Flow_M4_Transform(Body1->Position, Body1->Orientation);
				flow_m4 Transform2 = Flow_M4_Transform(Body2->Position, Body2->Orientation);

				flow_v3 WorldP1 = Flow_V4_Mul_M4(Flow_V4_From_V3(Contact->ContactPoint.LocalP1, flow_constant(1.0)), &Transform1).xyz;
				flow_v3 WorldP2 = Flow_V4_Mul_M4(Flow_V4_From_V3(Contact->ContactPoint.LocalP2, flow_constant(1.0)), &Transform2).xyz;

				//Separation along the penetration vector. Not just from each other
				flow_real Separation = Flow_V3_Dot(Flow_V3_Sub_V3(WorldP2, WorldP1), Constraint->Normal) + FLOW_PENETRATION_SLOP;
				if (Separation < 0.0f) {
					flow_m3 InvInertiaWorld1 = Flow_Body_Get_World_Inertia(Body1, &InvInertia1);
					flow_m3 InvInertiaWorld2 = Flow_Body_Get_World_Inertia(Body2, &InvInertia2);

					flow_v3 P = Flow_V3_Mul_S(Flow_V3_Add_V3(WorldP1, WorldP2), flow_constant(0.5));
					flow_v3 R1 = Flow_V3_Sub_V3(P, Flow_Body_Get_COM(Body1));
					flow_v3 R2 = Flow_V3_Sub_V3(P, Flow_Body_Get_COM(Body2));

					Flow_Contact_Constraint_Init(&Contact->NormalConstraint, R1, R2, InvMass1, InvMass2, &InvInertiaWorld1, &InvInertiaWorld2, Constraint->Normal);
					Flow_Contact_Constraint_Solve_Position(&Contact->NormalConstraint, Body1, Body2, InvMass1, InvMass2, Constraint->Normal, Separation, FLOW_BAUMGUARTE);
				}
			}
		}
	}
}

FLOWDEF void Flow_System_Set_Gravity(flow_system* System, flow_v3 Gravity) {
	System->Gravity = Gravity;
}

FLOWDEF void Flow_System_Get_Gravity(flow_system* System, flow_v3* Gravity) {
	*Gravity = System->Gravity;
}

FLOWDEF flow_body_id Flow_Create_Body(flow_system* System, const flow_body_create_info* CreateInfo) {
	flow_body_id ID = Flow_Body_Pool_Allocate(&System->BodyPool);
	flow_body* Body = Flow_Body_Pool_Get(&System->BodyPool, ID);

	Body->Position = CreateInfo->Position;
	Body->Orientation = CreateInfo->Orientation;
	Body->Scale = CreateInfo->Scale;
	Body->Type = CreateInfo->Type;
	Body->Collider = CreateInfo->Collider;
	Body->Friction = CreateInfo->Friction;
	Body->UserData = CreateInfo->UserData;

	return ID;
}

FLOWDEF flow_body* Flow_Get_Body(flow_system* System, flow_body_id ID) {
	flow_body* Body = Flow_Body_Pool_Get(&System->BodyPool, ID);
	return Body;
}

FLOWDEF void Flow_Delete_Body(flow_system* System, flow_body_id ID);

#ifdef FLOW_DEBUG_RENDERING

FLOWDEF flow_debug_renderer* Flow_Get_Renderer(flow_system* System) {
	return &System->DebugRenderer;
}

#endif

#ifdef FLOW_COMPILER_MSVC
#pragma warning(pop)
#endif

#ifdef FLOW_COMPILER_CLANG
#pragma clang diagnostic pop
#endif

#endif