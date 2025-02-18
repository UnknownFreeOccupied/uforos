/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_ROS_MSGS_CONVERSIONS_HPP
#define UFO_ROS_MSGS_CONVERSIONS_HPP

// UFO
#include <ufo/map/buffer.hpp>
#include <ufo/map/io.hpp>

// UFO ROS
#include <ufo_msgs/Map.h>

// STL
#include <type_traits>

namespace ufo_msgs
{
//
// ROS message type to UFO type
//

ufo::FileHeader msgToHeader(ufo_msgs::Map const& msg);

template <class Map>
void msgToUfo(ufo_msgs::Map const& msg, Map& map, bool propagate = true)
{
	if (msg.data.empty()) {
		return;
	}
	ufo::Buffer buffer;
	buffer.write(msg.data.data(),
	             msg.data.size() * sizeof(decltype(ufo_msgs::Map::data)::value_type));
	map.read(buffer, propagate);
}

//
// UFO type to ROS message type
//

template <class Map, class Predicates,
          typename = std::enable_if_t<!std::is_scalar_v<Predicates>>>
decltype(ufo_msgs::Map::data) ufoToMsg(Map const& map, Predicates const& predicates,
                                      unsigned int depth = 0, bool compress = false,
                                      ufo::mt_t map_types                      = 0,
                                      int       compression_acceleration_level = 1,
                                      int       compression_level              = 0)
{
	auto                         data = map.write(predicates, depth, compress, map_types,
	                                              compression_acceleration_level, compression_level);
	decltype(ufo_msgs::Map::data) ret;
	ret.resize(data.size());
	data.read(ret.data(), data.size());
	return ret;
}

template <class Map>
decltype(ufo_msgs::Map::data) ufoToMsg(Map const& map, unsigned int depth = 0,
                                      bool compress = false, ufo::mt_t map_types = 0,
                                      int compression_acceleration_level = 1,
                                      int compression_level              = 0)
{
	auto data = map.write(depth, compress, map_types, compression_acceleration_level,
	                      compression_level);
	decltype(ufo_msgs::Map::data) ret;
	ret.resize(data.size());
	data.read(ret.data(), data.size());
	return ret;
}

template <class Map>
decltype(ufo_msgs::Map::data) ufoToMsgModified(Map& map, bool compress = false,
                                              ufo::mt_t map_types                = 0,
                                              int compression_acceleration_level = 1,
                                              int compression_level              = 0)
{
	auto data = map.writeModified(compress, map_types, compression_acceleration_level,
	                              compression_level);
	decltype(ufo_msgs::Map::data) ret;
	ret.resize(data.size());
	data.read(ret.data(), data.size());
	return ret;
}

template <class Map>
decltype(ufo_msgs::Map::data) ufoToMsgResetModified(Map& map, bool compress = false,
                                                   ufo::mt_t map_types                = 0,
                                                   int compression_acceleration_level = 1,
                                                   int compression_level              = 0)
{
	auto data = map.writeModifiedAndReset(
	    compress, map_types, compression_acceleration_level, compression_level);
	decltype(ufo_msgs::Map::data) ret;
	ret.resize(data.size());
	data.read(ret.data(), data.size());
	return ret;
}

}  // namespace ufo_msgs

#endif  // UFO_ROS_MSGS_CONVERSIONS_HPP