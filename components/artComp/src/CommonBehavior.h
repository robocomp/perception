// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `CommonBehavior.ice'

#ifndef ____CommonBehavior_h__
#define ____CommonBehavior_h__

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

namespace RoboCompCommonBehavior
{

class CommonBehavior;

}

}

namespace RoboCompCommonBehavior
{

class CommonBehavior;
bool operator==(const CommonBehavior&, const CommonBehavior&);
bool operator<(const CommonBehavior&, const CommonBehavior&);

}

namespace IceInternal
{

::Ice::Object* upCast(::RoboCompCommonBehavior::CommonBehavior*);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompCommonBehavior::CommonBehavior*);

}

namespace RoboCompCommonBehavior
{

typedef ::IceInternal::Handle< ::RoboCompCommonBehavior::CommonBehavior> CommonBehaviorPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompCommonBehavior::CommonBehavior> CommonBehaviorPrx;

void __read(::IceInternal::BasicStream*, CommonBehaviorPrx&);
void __patch__CommonBehaviorPtr(void*, ::Ice::ObjectPtr&);

}

namespace RoboCompCommonBehavior
{

enum State
{
    Starting,
    Running
};

void __write(::IceInternal::BasicStream*, State);
void __read(::IceInternal::BasicStream*, State&);

struct Parameter
{
    bool editable;
    ::std::string value;
    ::std::string type;

    bool operator==(const Parameter&) const;
    bool operator<(const Parameter&) const;
    bool operator!=(const Parameter& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Parameter& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Parameter& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Parameter& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::map< ::std::string, ::RoboCompCommonBehavior::Parameter> ParameterList;
void __writeParameterList(::IceInternal::BasicStream*, const ParameterList&);
void __readParameterList(::IceInternal::BasicStream*, ParameterList&);

}

namespace IceProxy
{

namespace RoboCompCommonBehavior
{

class CommonBehavior : virtual public ::IceProxy::Ice::Object
{
public:

    ::Ice::Int getPeriod()
    {
        return getPeriod(0);
    }
    ::Ice::Int getPeriod(const ::Ice::Context& __ctx)
    {
        return getPeriod(&__ctx);
    }
    
private:

    ::Ice::Int getPeriod(const ::Ice::Context*);
    
public:

    void setPeriod(::Ice::Int period)
    {
        setPeriod(period, 0);
    }
    void setPeriod(::Ice::Int period, const ::Ice::Context& __ctx)
    {
        setPeriod(period, &__ctx);
    }
    
private:

    void setPeriod(::Ice::Int, const ::Ice::Context*);
    
public:

    ::Ice::Int timeAwake()
    {
        return timeAwake(0);
    }
    ::Ice::Int timeAwake(const ::Ice::Context& __ctx)
    {
        return timeAwake(&__ctx);
    }
    
private:

    ::Ice::Int timeAwake(const ::Ice::Context*);
    
public:

    void killYourSelf()
    {
        killYourSelf(0);
    }
    void killYourSelf(const ::Ice::Context& __ctx)
    {
        killYourSelf(&__ctx);
    }
    
private:

    void killYourSelf(const ::Ice::Context*);
    
public:

    ::RoboCompCommonBehavior::ParameterList getParameterList()
    {
        return getParameterList(0);
    }
    ::RoboCompCommonBehavior::ParameterList getParameterList(const ::Ice::Context& __ctx)
    {
        return getParameterList(&__ctx);
    }
    
private:

    ::RoboCompCommonBehavior::ParameterList getParameterList(const ::Ice::Context*);
    
public:

    void setParameterList(const ::RoboCompCommonBehavior::ParameterList& l)
    {
        setParameterList(l, 0);
    }
    void setParameterList(const ::RoboCompCommonBehavior::ParameterList& l, const ::Ice::Context& __ctx)
    {
        setParameterList(l, &__ctx);
    }
    
private:

    void setParameterList(const ::RoboCompCommonBehavior::ParameterList&, const ::Ice::Context*);
    
public:

    void reloadConfig()
    {
        reloadConfig(0);
    }
    void reloadConfig(const ::Ice::Context& __ctx)
    {
        reloadConfig(&__ctx);
    }
    
private:

    void reloadConfig(const ::Ice::Context*);
    
public:

    ::RoboCompCommonBehavior::State getState()
    {
        return getState(0);
    }
    ::RoboCompCommonBehavior::State getState(const ::Ice::Context& __ctx)
    {
        return getState(&__ctx);
    }
    
private:

    ::RoboCompCommonBehavior::State getState(const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CommonBehavior> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CommonBehavior*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<CommonBehavior*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace RoboCompCommonBehavior
{

class CommonBehavior : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual ::Ice::Int getPeriod(const ::Ice::Context*) = 0;

    virtual void setPeriod(::Ice::Int, const ::Ice::Context*) = 0;

    virtual ::Ice::Int timeAwake(const ::Ice::Context*) = 0;

    virtual void killYourSelf(const ::Ice::Context*) = 0;

    virtual ::RoboCompCommonBehavior::ParameterList getParameterList(const ::Ice::Context*) = 0;

    virtual void setParameterList(const ::RoboCompCommonBehavior::ParameterList&, const ::Ice::Context*) = 0;

    virtual void reloadConfig(const ::Ice::Context*) = 0;

    virtual ::RoboCompCommonBehavior::State getState(const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompCommonBehavior
{

class CommonBehavior : virtual public ::IceDelegate::RoboCompCommonBehavior::CommonBehavior,
                       virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual ::Ice::Int getPeriod(const ::Ice::Context*);

    virtual void setPeriod(::Ice::Int, const ::Ice::Context*);

    virtual ::Ice::Int timeAwake(const ::Ice::Context*);

    virtual void killYourSelf(const ::Ice::Context*);

    virtual ::RoboCompCommonBehavior::ParameterList getParameterList(const ::Ice::Context*);

    virtual void setParameterList(const ::RoboCompCommonBehavior::ParameterList&, const ::Ice::Context*);

    virtual void reloadConfig(const ::Ice::Context*);

    virtual ::RoboCompCommonBehavior::State getState(const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace RoboCompCommonBehavior
{

class CommonBehavior : virtual public ::IceDelegate::RoboCompCommonBehavior::CommonBehavior,
                       virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual ::Ice::Int getPeriod(const ::Ice::Context*);

    virtual void setPeriod(::Ice::Int, const ::Ice::Context*);

    virtual ::Ice::Int timeAwake(const ::Ice::Context*);

    virtual void killYourSelf(const ::Ice::Context*);

    virtual ::RoboCompCommonBehavior::ParameterList getParameterList(const ::Ice::Context*);

    virtual void setParameterList(const ::RoboCompCommonBehavior::ParameterList&, const ::Ice::Context*);

    virtual void reloadConfig(const ::Ice::Context*);

    virtual ::RoboCompCommonBehavior::State getState(const ::Ice::Context*);
};

}

}

namespace RoboCompCommonBehavior
{

class CommonBehavior : virtual public ::Ice::Object
{
public:

    typedef CommonBehaviorPrx ProxyType;
    typedef CommonBehaviorPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::Ice::Int getPeriod(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getPeriod(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setPeriod(::Ice::Int, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setPeriod(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::Int timeAwake(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___timeAwake(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void killYourSelf(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___killYourSelf(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompCommonBehavior::ParameterList getParameterList(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getParameterList(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setParameterList(const ::RoboCompCommonBehavior::ParameterList&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setParameterList(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void reloadConfig(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___reloadConfig(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompCommonBehavior::State getState(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getState(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
