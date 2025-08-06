@echo off

setlocal enabledelayedexpansion

set base_path=%~dp0
set bin_path=%base_path%\bin
set base_lib_path=%base_path%\..\base

set build_debug=0
set build_clang=0
set build_asan=0

:cmd_line
if "%~1"=="" goto end_cmd_line

if /i "%~1"=="-debug" (
	set build_debug=1
)

if /i "%~1"=="-clang" (
	set build_clang=1
)

if /i "%~1"=="-asan" (
	set build_asan=1
)

if /i "%~1"=="-base_lib_path" (
	set base_lib_path="%~2"
	shift
)

shift
goto cmd_line
:end_cmd_line

set parameters=-bin_path %bin_path%
if %build_debug% == 1 (
	set parameters=%parameters% -debug
)

call %base_lib_path%\build.bat %parameters%

if not exist "%bin_path%" (
	mkdir "%bin_path%"
)

for /f "delims=" %%i in ('where clang') do (
	set "clang_path=%%~dpi"
)
set clang_path=%clang_path%..\lib\clang\21\lib\windows\

if %build_asan% == 1 (
	if %build_clang% == 1 (
		xcopy "%clang_path%*" "%bin_path%\" /E /I /H /Y >nul 2>&1
	)

	if %build_clang% == 0 (
		del /q "%bin_path%\clang_rt*" >nul 2>&1
	)
)

set app_includes=-I%base_path% -I%base_lib_path%\code
set app_defines=-DFLOW_IMPLEMENTATION

if %build_debug% == 1 (
	set app_defines=%app_defines% -DFLOW_DEBUG
)

set msvc_optimized_flag=/Od
set clang_optimized_flag=-O0
if %build_debug% == 0 (
	set msvc_optimized_flag=/O2
	set clang_optimized_flag=-O3
)

set clang_dll=-shared
set clang_compile_only=-c
set clang_warnings=-Wall -Werror
set clang_flags=-g -gcodeview %clang_optimized_flag%
set clang_out=-o 
set clang_link=

set msvc_dll=-LD
set msvc_compile_only=/c
set msvc_warnings=/Wall /WX /wd4514 /wd5045 /wd4668 /wd4255 /wd4820 /wd5246
set msvc_flags=/nologo /FC /Z7 /experimental:c11atomics %msvc_optimized_flag%
set msvc_out=/out:
set msvc_link=/link /opt:ref /incremental:no

if %build_asan% == 1 (
	set clang_flags=%clang_flags% -fsanitize=address
	set msvc_flags=%msvc_flags% -fsanitize=address
)

if %build_clang% == 1 (
	set obj_out=-o
	set compile_only=%clang_compile_only%
	set compile_warnings=%clang_warnings%
	set compile_flags=%clang_flags% 
	set compile_out=%clang_out%
	set compile_dll=%clang_dll%
	set compile_link=%clang_link%
	set compiler_c=clang -std=c99
	set compiler_cpp=clang++ -std=c++14
)

if %build_clang% == 0 (
	set obj_out=-Fo:
	set compile_only=%msvc_compile_only%
	set compile_warnings=%msvc_warnings%
	set compile_flags=%msvc_flags%
	set compile_out=%msvc_out%
	set compile_dll=%msvc_dll%
	set compile_link=%msvc_link%
	set compiler_c=cl /std:c11
	set compiler_cpp=cl /std:c++14
)

pushd "%bin_path%"
	%compiler_c% %compile_flags% %compile_warnings% %app_defines% %app_includes% "%base_path%/tests/c_tests.c" %compile_link% %compile_out%c_tests.exe
	%compiler_cpp% %compile_flags% %compile_warnings% %app_defines% %app_includes% "%base_path%/tests/cpp_tests.cpp" %compile_link% %compile_out%cpp_tests.exe

	%compiler_c% %compile_flags% %compile_warnings% %app_defines% %app_includes% -DFLOW_USE_F64 "%base_path%/tests/c_tests.c" %compile_link% %compile_out%c_tests64.exe
	%compiler_cpp% %compile_flags% %compile_warnings% %app_defines% %app_includes% -DFLOW_USE_F64 "%base_path%/tests/cpp_tests.cpp" %compile_link% %compile_out%cpp_tests64.exe
popd