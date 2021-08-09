/*
 * uavcan.h
 *
 *  Created on: 23 Jul 2021
 *      Author: User
 */

#ifndef UAVCAN_H_
#define UAVCAN_H_

#include "canard.h"
#include "o1heap.h"

#include <uavcan/node/Heartbeat_1_0.h>
//#include <uavcan/node/GetInfo_1_0.h>
//#include <uavcan/node/ExecuteCommand_1_1.h>
//#include <uavcan/node/port/List_0_1.h>
//#include <uavcan/_register/Access_1_0.h>
//#include <uavcan/_register/List_1_0.h>
//#include <uavcan/pnp/NodeIDAllocationData_2_0.h>

#define FRAMES_PER_ITER 1000
#define uavcan_si_sample_temperature_Scalar_1_0_PORT_ID 1001
#define uavcan_si_sample_pressure_Scalar_1_0_PORT_ID 1002

struct canardState;

typedef CanardMicrosecond (*monotonicTimeSource)(void);
typedef void (*receivedTransferCallback)(struct canardState* const, const CanardTransfer* const);

// A struct containing several relevant variables, easier than passing a dozen arguments to each function
typedef struct canardState
{
    CanardMicrosecond startedAt;

    CanardInstance  canard;

    O1HeapInstance* heap;

    monotonicTimeSource getMonotonicMicroseconds;

    receivedTransferCallback transferCallback;

    uint64_t uavcan_node_heartbeat_transfer_id;

} CanardState;

typedef struct uavcanMessage
{
	CanardPortID portId;
	CanardPriority priority;
	uint64_t timeout;
	uint64_t transferId;
}UavcanMessage;

int8_t uavcanInit(CanardState *state, uint8_t nodeId, uint32_t bitrate);
void canardProcessTx(CanardState* const state);
void canardProcessRx(CanardState* const state);
void PublishHeartbeat(CanardState* const state, const CanardMicrosecond currentTime);

#endif /* UAVCAN_H_ */
