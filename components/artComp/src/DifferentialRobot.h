// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `DifferentialRobot.ice'

#ifndef ____DifferentialRobot_h__
#define ____DifferentialRobot_h__

#include <Ice/LocalObjectF.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/Outgoing.h>
#include <Ice/Incoming.h>
#include <Ice/Direct.h>
#include <Ice/UserExceptionFactory.h>
#include <Ice/FactoryTable.h>
#include <Ice/StreamF.h>
#include <Ice/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 303
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RoboCompDifferentialRobot
{

class DifferentialRobot;

}

}

namespace RoboCompDifferentialRobot
{

class DifferentialRobot;
bool operator==(const DifferentialRobot&, const DifferentialRobot&);
bool operator<(const DifferentialRobot&, const DifferentialRobot&);

}

namespace IceInternal
{

::Ice::Object* upCast(::RoboCompDifferentialRobot::DifferentialRobot*);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompDifferentialRobot::DifferentialRobot*);

}

namespace RoboCompDifferentialRobot
{

typedef ::IceInternal::Handle< ::RoboCompDifferentialRobot::DifferentialRobot> DifferentialRobotPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompDifferentialRobot::DifferentialRobot> DifferentialRobotPrx;

void __read(::IceInternal::BasicStream*, DifferentialRobotPrx&);
void __patch__DifferentialRobotPtr(void*, ::Ice::ObjectPtr&);

}

namespace RoboCompDifferentialRobot
{

class HardwareFailedException : public ::Ice::UserException
{
public:

    HardwareFailedException() {}
    explicit HardwareFailedException(const ::std::string&);
    virtual ~HardwareFailedException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string what;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

static HardwareFailedException __HardwareFailedException_init;

struct TMechParams
{
    ::Ice::Int wheelRadius;
    ::Ice::Int axisLength;
    ::Ice::Int encoderSteps;
    ::Ice::Int gearRatio;
    ::Ice::Float temp;
    ::Ice::Float maxVelAdv;
    ::Ice::Float maxVelRot;
    ::std::string device;
    ::std::string handler;

    bool operator==(const TMechParams&) const;
    bool operator<(const TMechParams&) const;
    bool operator!=(const TMechParams& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const TMechParams& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const TMechParams& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const TMechParams& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct TBaseState
{
    bool isMoving;
    ::Ice::Float x;
    ::Ice::Float correctedX;
    ::Ice::Float z;
    ::Ice::Float correctedZ;
    ::Ice::Float alpha;
    ::Ice::Float correctedAlpha;
    ::Ice::Float advV;
    ::Ice::Float rotV;
    ::Ice::Float adv;
    ::Ice::Float rot;

    bool operator==(const TBaseState&) const;
    bool operator<(const TBaseState&) const;
    bool operator!=(const TBaseState& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const TBaseState& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const TBaseState& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const TBaseState& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

}

namespace IceProxy
{

namespace RoboCompDifferentialRobot
{

class DifferentialRobot : virtual public ::IceProxy::Ice::Object
{
public:

    void getBaseState(::RoboCompDifferentialRobot::TBaseState& state)
    {
        getBaseState(state, 0);
    }
    void getBaseState(::RoboCompDifferentialRobot::TBaseState& state, const ::Ice::Context& __ctx)
    {
        getBaseState(state, &__ctx);
    }
    
private:

    void getBaseState(::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Context*);
    
public:

    void getBasePose(::Ice::Int& x, ::Ice::Int& z, ::Ice::Float& alpha)
    {
        getBasePose(x, z, alpha, 0);
    }
    void getBasePose(::Ice::Int& x, ::Ice::Int& z, ::Ice::Float& alpha, const ::Ice::Context& __ctx)
    {
        getBasePose(x, z, alpha, &__ctx);
    }
    
private:

    void getBasePose(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);
    
public:

    void setSpeedBase(::Ice::Float adv, ::Ice::Float rot)
    {
        setSpeedBase(adv, rot, 0);
    }
    void setSpeedBase(::Ice::Float adv, ::Ice::Float rot, const ::Ice::Context& __ctx)
    {
        setSpeedBase(adv, rot, &__ctx);
    }
    
private:

    void setSpeedBase(::Ice::Float, ::Ice::Float, const ::Ice::Context*);
    
public:

    void stopBase()
    {
        stopBase(0);
    }
    void stopBase(const ::Ice::Context& __ctx)
    {
        stopBase(&__ctx);
    }
    
private:

    void stopBase(const ::Ice::Context*);
    
public:

    void resetOdometer()
    {
        resetOdometer(0);
    }
    void resetOdometer(const ::Ice::Context& __ctx)
    {
        resetOdometer(&__ctx);
    }
    
private:

    void resetOdometer(const ::Ice::Context*);
    
public:

    void setOdometer(const ::RoboCompDifferentialRobot::TBaseState& state)
    {
        setOdometer(state, 0);
    }
    void setOdometer(const ::RoboCompDifferentialRobot::TBaseState& state, const ::Ice::Context& __ctx)
    {
        setOdometer(state, &__ctx);
    }
    
private:

    void setOdometer(const ::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Context*);
    
public:

    void setOdometerPose(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha)
    {
        setOdometerPose(x, z, alpha, 0);
    }
    void setOdometerPose(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha, const ::Ice::Context& __ctx)
    {
        setOdometerPose(x, z, alpha, &__ctx);
    }
    
private:

    void setOdometerPose(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);
    
public:

    void correctOdometer(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha)
    {
        correctOdometer(x, z, alpha, 0);
    }
    void correctOdometer(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha, const ::Ice::Context& __ctx)
    {
        correctOdometer(x, z, alpha, &__ctx);
    }
    
private:

    void correctOdometer(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DifferentialRobot> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DifferentialRobot*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<DifferentialRobot*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace IceDelegate
{

namespace RoboCompDifferentialRobot
{

class DifferentialRobot : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void getBaseState(::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Context*) = 0;

    virtual void getBasePose(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*) = 0;

    virtual void setSpeedBase(::Ice::Float, ::Ice::Float, const ::Ice::Context*) = 0;

    virtual void stopBase(const ::Ice::Context*) = 0;

    virtual void resetOdometer(const ::Ice::Context*) = 0;

    virtual void setOdometer(const ::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Context*) = 0;

    virtual void setOdometerPose(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*) = 0;

    virtual void correctOdometer(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompDifferentialRobot
{

class DifferentialRobot : virtual public ::IceDelegate::RoboCompDifferentialRobot::DifferentialRobot,
                          virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void getBaseState(::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Context*);

    virtual void getBasePose(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);

    virtual void setSpeedBase(::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void stopBase(const ::Ice::Context*);

    virtual void resetOdometer(const ::Ice::Context*);

    virtual void setOdometer(const ::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Context*);

    virtual void setOdometerPose(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);

    virtual void correctOdometer(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace RoboCompDifferentialRobot
{

class DifferentialRobot : virtual public ::IceDelegate::RoboCompDifferentialRobot::DifferentialRobot,
                          virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void getBaseState(::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Context*);

    virtual void getBasePose(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);

    virtual void setSpeedBase(::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void stopBase(const ::Ice::Context*);

    virtual void resetOdometer(const ::Ice::Context*);

    virtual void setOdometer(const ::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Context*);

    virtual void setOdometerPose(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);

    virtual void correctOdometer(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);
};

}

}

namespace RoboCompDifferentialRobot
{

class DifferentialRobot : virtual public ::Ice::Object
{
public:

    typedef DifferentialRobotPrx ProxyType;
    typedef DifferentialRobotPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void getBaseState(::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getBaseState(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getBasePose(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getBasePose(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setSpeedBase(::Ice::Float, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setSpeedBase(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void stopBase(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___stopBase(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void resetOdometer(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___resetOdometer(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setOdometer(const ::RoboCompDifferentialRobot::TBaseState&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setOdometer(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setOdometerPose(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setOdometerPose(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void correctOdometer(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___correctOdometer(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
