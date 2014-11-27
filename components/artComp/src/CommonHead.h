// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `CommonHead.ice'

#ifndef ____CommonHead_h__
#define ____CommonHead_h__

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
#include <JointMotor.h>
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

namespace RoboCompCommonHead
{

class CommonHead;

}

}

namespace RoboCompCommonHead
{

class CommonHead;
bool operator==(const CommonHead&, const CommonHead&);
bool operator<(const CommonHead&, const CommonHead&);

}

namespace IceInternal
{

::Ice::Object* upCast(::RoboCompCommonHead::CommonHead*);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompCommonHead::CommonHead*);

}

namespace RoboCompCommonHead
{

typedef ::IceInternal::Handle< ::RoboCompCommonHead::CommonHead> CommonHeadPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompCommonHead::CommonHead> CommonHeadPrx;

void __read(::IceInternal::BasicStream*, CommonHeadPrx&);
void __patch__CommonHeadPtr(void*, ::Ice::ObjectPtr&);

}

namespace RoboCompCommonHead
{

typedef ::std::map< ::std::string, ::RoboCompJointMotor::MotorParams> dmotorParams;
void __writedmotorParams(::IceInternal::BasicStream*, const dmotorParams&);
void __readdmotorParams(::IceInternal::BasicStream*, dmotorParams&);

struct THeadParams
{
    ::RoboCompCommonHead::dmotorParams motorsParams;
    ::std::string model;

    bool operator==(const THeadParams&) const;
    bool operator<(const THeadParams&) const;
    bool operator!=(const THeadParams& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const THeadParams& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const THeadParams& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const THeadParams& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::map< ::std::string, ::RoboCompJointMotor::MotorState> dmotorsState;
void __writedmotorsState(::IceInternal::BasicStream*, const dmotorsState&);
void __readdmotorsState(::IceInternal::BasicStream*, dmotorsState&);

struct THeadState
{
    ::RoboCompCommonHead::dmotorsState motorsState;
    bool isMoving;

    bool operator==(const THeadState&) const;
    bool operator<(const THeadState&) const;
    bool operator!=(const THeadState& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const THeadState& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const THeadState& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const THeadState& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

}

namespace IceProxy
{

namespace RoboCompCommonHead
{

class CommonHead : virtual public ::IceProxy::Ice::Object
{
public:

    void resetHead()
    {
        resetHead(0);
    }
    void resetHead(const ::Ice::Context& __ctx)
    {
        resetHead(&__ctx);
    }
    
private:

    void resetHead(const ::Ice::Context*);
    
public:

    void stopHead()
    {
        stopHead(0);
    }
    void stopHead(const ::Ice::Context& __ctx)
    {
        stopHead(&__ctx);
    }
    
private:

    void stopHead(const ::Ice::Context*);
    
public:

    void setPanLeft(::Ice::Float pan)
    {
        setPanLeft(pan, 0);
    }
    void setPanLeft(::Ice::Float pan, const ::Ice::Context& __ctx)
    {
        setPanLeft(pan, &__ctx);
    }
    
private:

    void setPanLeft(::Ice::Float, const ::Ice::Context*);
    
public:

    void setPanRight(::Ice::Float pan)
    {
        setPanRight(pan, 0);
    }
    void setPanRight(::Ice::Float pan, const ::Ice::Context& __ctx)
    {
        setPanRight(pan, &__ctx);
    }
    
private:

    void setPanRight(::Ice::Float, const ::Ice::Context*);
    
public:

    void setTilt(::Ice::Float tilt)
    {
        setTilt(tilt, 0);
    }
    void setTilt(::Ice::Float tilt, const ::Ice::Context& __ctx)
    {
        setTilt(tilt, &__ctx);
    }
    
private:

    void setTilt(::Ice::Float, const ::Ice::Context*);
    
public:

    void setNeck(::Ice::Float neck)
    {
        setNeck(neck, 0);
    }
    void setNeck(::Ice::Float neck, const ::Ice::Context& __ctx)
    {
        setNeck(neck, &__ctx);
    }
    
private:

    void setNeck(::Ice::Float, const ::Ice::Context*);
    
public:

    void saccadic2DLeft(::Ice::Float leftPan, ::Ice::Float tilt)
    {
        saccadic2DLeft(leftPan, tilt, 0);
    }
    void saccadic2DLeft(::Ice::Float leftPan, ::Ice::Float tilt, const ::Ice::Context& __ctx)
    {
        saccadic2DLeft(leftPan, tilt, &__ctx);
    }
    
private:

    void saccadic2DLeft(::Ice::Float, ::Ice::Float, const ::Ice::Context*);
    
public:

    void saccadic2DRight(::Ice::Float rightPan, ::Ice::Float tilt)
    {
        saccadic2DRight(rightPan, tilt, 0);
    }
    void saccadic2DRight(::Ice::Float rightPan, ::Ice::Float tilt, const ::Ice::Context& __ctx)
    {
        saccadic2DRight(rightPan, tilt, &__ctx);
    }
    
private:

    void saccadic2DRight(::Ice::Float, ::Ice::Float, const ::Ice::Context*);
    
public:

    void saccadic3D(::Ice::Float leftPan, ::Ice::Float rightPan, ::Ice::Float tilt)
    {
        saccadic3D(leftPan, rightPan, tilt, 0);
    }
    void saccadic3D(::Ice::Float leftPan, ::Ice::Float rightPan, ::Ice::Float tilt, const ::Ice::Context& __ctx)
    {
        saccadic3D(leftPan, rightPan, tilt, &__ctx);
    }
    
private:

    void saccadic3D(::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);
    
public:

    void saccadic4D(::Ice::Float leftPan, ::Ice::Float rightPan, ::Ice::Float tilt, ::Ice::Float neck)
    {
        saccadic4D(leftPan, rightPan, tilt, neck, 0);
    }
    void saccadic4D(::Ice::Float leftPan, ::Ice::Float rightPan, ::Ice::Float tilt, ::Ice::Float neck, const ::Ice::Context& __ctx)
    {
        saccadic4D(leftPan, rightPan, tilt, neck, &__ctx);
    }
    
private:

    void saccadic4D(::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);
    
public:

    void setNMotorsPosition(const ::RoboCompJointMotor::MotorGoalPositionList& listGoals)
    {
        setNMotorsPosition(listGoals, 0);
    }
    void setNMotorsPosition(const ::RoboCompJointMotor::MotorGoalPositionList& listGoals, const ::Ice::Context& __ctx)
    {
        setNMotorsPosition(listGoals, &__ctx);
    }
    
private:

    void setNMotorsPosition(const ::RoboCompJointMotor::MotorGoalPositionList&, const ::Ice::Context*);
    
public:

    ::RoboCompCommonHead::THeadParams getHeadParams()
    {
        return getHeadParams(0);
    }
    ::RoboCompCommonHead::THeadParams getHeadParams(const ::Ice::Context& __ctx)
    {
        return getHeadParams(&__ctx);
    }
    
private:

    ::RoboCompCommonHead::THeadParams getHeadParams(const ::Ice::Context*);
    
public:

    void getHeadState(::RoboCompCommonHead::THeadState& hState)
    {
        getHeadState(hState, 0);
    }
    void getHeadState(::RoboCompCommonHead::THeadState& hState, const ::Ice::Context& __ctx)
    {
        getHeadState(hState, &__ctx);
    }
    
private:

    void getHeadState(::RoboCompCommonHead::THeadState&, const ::Ice::Context*);
    
public:

    bool isMovingHead()
    {
        return isMovingHead(0);
    }
    bool isMovingHead(const ::Ice::Context& __ctx)
    {
        return isMovingHead(&__ctx);
    }
    
private:

    bool isMovingHead(const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<CommonHead> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonHead> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonHead*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<CommonHead*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace RoboCompCommonHead
{

class CommonHead : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void resetHead(const ::Ice::Context*) = 0;

    virtual void stopHead(const ::Ice::Context*) = 0;

    virtual void setPanLeft(::Ice::Float, const ::Ice::Context*) = 0;

    virtual void setPanRight(::Ice::Float, const ::Ice::Context*) = 0;

    virtual void setTilt(::Ice::Float, const ::Ice::Context*) = 0;

    virtual void setNeck(::Ice::Float, const ::Ice::Context*) = 0;

    virtual void saccadic2DLeft(::Ice::Float, ::Ice::Float, const ::Ice::Context*) = 0;

    virtual void saccadic2DRight(::Ice::Float, ::Ice::Float, const ::Ice::Context*) = 0;

    virtual void saccadic3D(::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*) = 0;

    virtual void saccadic4D(::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*) = 0;

    virtual void setNMotorsPosition(const ::RoboCompJointMotor::MotorGoalPositionList&, const ::Ice::Context*) = 0;

    virtual ::RoboCompCommonHead::THeadParams getHeadParams(const ::Ice::Context*) = 0;

    virtual void getHeadState(::RoboCompCommonHead::THeadState&, const ::Ice::Context*) = 0;

    virtual bool isMovingHead(const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompCommonHead
{

class CommonHead : virtual public ::IceDelegate::RoboCompCommonHead::CommonHead,
                   virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void resetHead(const ::Ice::Context*);

    virtual void stopHead(const ::Ice::Context*);

    virtual void setPanLeft(::Ice::Float, const ::Ice::Context*);

    virtual void setPanRight(::Ice::Float, const ::Ice::Context*);

    virtual void setTilt(::Ice::Float, const ::Ice::Context*);

    virtual void setNeck(::Ice::Float, const ::Ice::Context*);

    virtual void saccadic2DLeft(::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void saccadic2DRight(::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void saccadic3D(::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void saccadic4D(::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void setNMotorsPosition(const ::RoboCompJointMotor::MotorGoalPositionList&, const ::Ice::Context*);

    virtual ::RoboCompCommonHead::THeadParams getHeadParams(const ::Ice::Context*);

    virtual void getHeadState(::RoboCompCommonHead::THeadState&, const ::Ice::Context*);

    virtual bool isMovingHead(const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace RoboCompCommonHead
{

class CommonHead : virtual public ::IceDelegate::RoboCompCommonHead::CommonHead,
                   virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void resetHead(const ::Ice::Context*);

    virtual void stopHead(const ::Ice::Context*);

    virtual void setPanLeft(::Ice::Float, const ::Ice::Context*);

    virtual void setPanRight(::Ice::Float, const ::Ice::Context*);

    virtual void setTilt(::Ice::Float, const ::Ice::Context*);

    virtual void setNeck(::Ice::Float, const ::Ice::Context*);

    virtual void saccadic2DLeft(::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void saccadic2DRight(::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void saccadic3D(::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void saccadic4D(::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void setNMotorsPosition(const ::RoboCompJointMotor::MotorGoalPositionList&, const ::Ice::Context*);

    virtual ::RoboCompCommonHead::THeadParams getHeadParams(const ::Ice::Context*);

    virtual void getHeadState(::RoboCompCommonHead::THeadState&, const ::Ice::Context*);

    virtual bool isMovingHead(const ::Ice::Context*);
};

}

}

namespace RoboCompCommonHead
{

class CommonHead : virtual public ::Ice::Object
{
public:

    typedef CommonHeadPrx ProxyType;
    typedef CommonHeadPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void resetHead(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___resetHead(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void stopHead(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___stopHead(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setPanLeft(::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setPanLeft(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setPanRight(::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setPanRight(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setTilt(::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setTilt(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setNeck(::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setNeck(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void saccadic2DLeft(::Ice::Float, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___saccadic2DLeft(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void saccadic2DRight(::Ice::Float, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___saccadic2DRight(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void saccadic3D(::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___saccadic3D(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void saccadic4D(::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___saccadic4D(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setNMotorsPosition(const ::RoboCompJointMotor::MotorGoalPositionList&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setNMotorsPosition(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompCommonHead::THeadParams getHeadParams(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getHeadParams(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getHeadState(::RoboCompCommonHead::THeadState&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getHeadState(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual bool isMovingHead(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___isMovingHead(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
