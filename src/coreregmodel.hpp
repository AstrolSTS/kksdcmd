//
//  Copyright (c) 2022 plan44.ch / Lukas Zeller, Zurich, Switzerland
//
//  Author: Lukas Zeller <luz@plan44.ch>
//
//  This file is part of kksdcmd.
//
//  kksdcmd is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  kksdcmd is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with kksdcmd. If not, see <http://www.gnu.org/licenses/>.
//

#ifndef __kksdcmd__coreregmodel__
#define __kksdcmd__coreregmodel__

#include "p44utils_common.hpp"
#include "corespiproto.hpp"

using namespace std;

namespace p44 {

  class CoreRegModel : public P44LoggingObj
  {
    typedef P44LoggingObj inherited;

  public:

    CoreRegModel();
    virtual ~CoreRegModel();


  };
  typedef boost::intrusive_ptr<CoreRegModel> CoreRegModelPtr;


} // namespace p44

#endif // __kksdcmd__coreregmodel__
