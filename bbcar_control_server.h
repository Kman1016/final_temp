/*
 * Generated by erpcgen 1.10.0 on Sun May 28 20:10:47 2023.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#if !defined(_bbcar_control_server_h_)
#define _bbcar_control_server_h_

#ifdef __cplusplus
#include "erpc_server.hpp"
#include "erpc_codec.hpp"
extern "C"
{
#include "bbcar_control.h"
#include <stdint.h>
#include <stdbool.h>
}

#if 11000 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif


/*!
 * @brief Service subclass for BBCarService.
 */
class BBCarService_service : public erpc::Service
{
public:
    BBCarService_service() : Service(kBBCarService_service_id) {}

    /*! @brief Call the correct server shim based on method unique ID. */
    virtual erpc_status_t handleInvocation(uint32_t methodId, uint32_t sequence, erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory);

private:
    /*! @brief Server shim for stop of BBCarService interface. */
    erpc_status_t stop_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);

    /*! @brief Server shim for goStraight of BBCarService interface. */
    erpc_status_t goStraight_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);

    /*! @brief Server shim for turn of BBCarService interface. */
    erpc_status_t turn_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);

    /*! @brief Server shim for ctl_mode of BBCarService interface. */
    erpc_status_t ctl_mode_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);

    /*! @brief Server shim for RemoteControlAction of BBCarService interface. */
    erpc_status_t RemoteControlAction_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);

    /*! @brief Server shim for RemoteTuneSpeed of BBCarService interface. */
    erpc_status_t RemoteTuneSpeed_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);

    /*! @brief Server shim for RemoteShowSpeed of BBCarService interface. */
    erpc_status_t RemoteShowSpeed_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);

    /*! @brief Server shim for RemoteShowPattern of BBCarService interface. */
    erpc_status_t RemoteShowPattern_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);
};

extern "C" {
#else
#include "bbcar_control.h"
#endif // __cplusplus

typedef void * erpc_service_t;

/*! @brief Return BBCarService_service service object. */
erpc_service_t create_BBCarService_service(void);

/*! @brief Destroy BBCarService_service service object. */
void destroy_BBCarService_service(erpc_service_t service);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _bbcar_control_server_h_
