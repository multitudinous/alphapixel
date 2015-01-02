# - Find Cairo
# Find the Cairo libraries
#
#  This module defines the following variables:
#     CAIRO_FOUND        - true if CAIRO_INCLUDE_DIR & CAIRO_LIBRARY are found
#     CAIRO_LIBRARIES    - Set when CAIRO_LIBRARY is found
#     CAIRO_INCLUDE_DIRS - Set when CAIRO_INCLUDE_DIR is found
#
#     CAIRO_INCLUDE_DIR  - where to find cairo.h, etc.
#     CAIRO_LIBRARY      - the Cairo library
#

#=============================================================================
# Copyright 2013 Marc-Andre Moreau <marcandre.moreau@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#=============================================================================

find_path(CAIRO_INCLUDE_DIR NAMES cairo.h PATH_SUFFIXES cairo)

find_library(CAIRO_LIBRARY NAMES cairo)

find_package_handle_standard_args(cairo DEFAULT_MSG PIXMAN_LIBRARY PIXMAN_INCLUDE_DIR)

if(CAIRO_FOUND)
	set(CAIRO_LIBRARIES ${CAIRO_LIBRARY})
	set(CAIRO_INCLUDE_DIRS ${CAIRO_INCLUDE_DIR}/cairo)
endif()

mark_as_advanced(CAIRO_INCLUDE_DIR CAIRO_LIBRARY)
