#ifndef FLOW_H
#define FLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef FLOW_STATIC_DEF
#define FLOWDEF static
#else
#define FLOWDEF extern
#endif

/* Generate unique names */
#define flow_static_assert_line(condition, line) \
typedef char static_assert_##line[(condition) ? 1 : -1]

#define flow_static_assert(condition) flow_static_assert_line(condition, __LINE__)

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

typedef struct {
	flow_real Data[3];
} flow_v3;

typedef struct {
	flow_real Data[4];
} flow_quat;

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

typedef struct {
	flow_v3   Position;
	flow_quat Orientation;
	flow_v3   Scale;
	void* 	  UserData;
} flow_body_create_info;

typedef struct {
	flow_v3   Position;
	flow_quat Orientation;
	flow_v3   Scale;
	void* 	  UserData;
} flow_body;

typedef flow_u64 flow_body_id;

FLOWDEF flow_body_id Flow_Create_Body(flow_system* System, const flow_body_create_info* CreateInfo);
FLOWDEF flow_body*   Flow_Get_Body(flow_system* System, flow_body_id ID);
FLOWDEF void 		 Flow_Delete_Body(flow_system* System, flow_body_id ID);

#ifdef __cplusplus
}
#endif

#endif

#ifdef FLOW_IMPLEMENTATION

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

#endif