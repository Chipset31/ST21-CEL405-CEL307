/*
 * uavcan.c
 *
 *  Created on: 23 Jul 2021
 *      Author: User
 */

#include <uavcan.h>

#include "bxcan.h"
#include "canard.h"
#include "o1heap.h"

#ifdef USE_HAL_DRIVER
#include <stm32l4xx_hal.h>
#endif

// We don't use the HAL_CAN drivers, but we need to make sure HAL_GetTick is available.
#ifdef USE_HAL_DRIVER
static CanardMicrosecond getMicroseconds(void)
{
	return (CanardMicrosecond)(HAL_GetTick()*1000);
}
#endif

/*
 * @brief 	A wrapper for libcanard to use the memory allocation function of the O1Heap library.
 * 			libcanard requires a constant-complexity deterministic dynamic memory allocator. Most
 * 			standard C heap implementations are not constant-complexity, so the libcanard documentation
 * 			recommends O1Heap.
 * @param 	ins		The instance of libcanard, requires that a pointer to the O1Heap instance
 * 					is available through the user_reference field of the instance struct.
 * @param	amount	The amount of bytes to allocate.
 * @retval 	None
 */
static void* canardAllocate(CanardInstance* const ins, const size_t amount)
{
	O1HeapInstance* const heap = ((CanardState*) ins->user_reference)->heap;
	assert(o1heapDoInvariantsHold(heap));
	return o1heapAllocate(heap, amount);
}

/*
 * @brief 	A wrapper for libcanard to use the memory free function of the O1Heap library.
 * 			libcanard requires a constant-complexity deterministic dynamic memory allocator. Most
 * 			standard C heap implementations are not constant-complexity, so the libcanard documentation
 *			recommends O1Heap.
 * @param 	ins		The instance of libcanard, requires that a pointer to the O1Heap instance
 * 					is available through the user_reference field of the instance struct.
 * @param	pointer	A pointer to the memory which is to be freed.
 * @retval 	None
 */
static void canardFree(CanardInstance* const ins, void* const pointer)
{
	O1HeapInstance* const heap = ((CanardState*) ins->user_reference)->heap;
	o1heapFree(heap, pointer);
}

int8_t uavcanInit(CanardState* const state, uint8_t nodeId, uint32_t bitrate)
{

	/*bxCAN driver initialisation*/

	BxCANTimings timings;
	bxCANComputeTimings(HAL_RCC_GetPCLK1Freq(), bitrate, &timings);

	if(bxCANConfigure(0, timings, false)!=true)
	{
		return -1;
	}

	/*o1heap initialisation*/

	// A simple node like this one typically does not require more than 4 KiB of heap and 4 KiB of stack.
	// For the background and related theory refer to the following resources:
	// - https://github.com/UAVCAN/libcanard/blob/master/README.md
	// - https://github.com/pavel-kirienko/o1heap/blob/master/README.md
	// - https://forum.uavcan.org/t/uavcanv1-libcanard-nunavut-templates-memory-usage-concerns/1118/4?u=pavel.kirienko
	_Alignas(O1HEAP_ALIGNMENT) static uint8_t heap_arena[1024 * 4] = {0};

	// If you are using an RTOS or another multithreaded environment, pass critical section enter/leave functions
	// in the last two arguments instead of NULL.
	state->heap = o1heapInit(heap_arena, sizeof(heap_arena), NULL, NULL);
	if (state->heap == NULL)
	{
		return -1;
	}

	/*libcanard initialisation*/

	//Fills the instance struct with default values and the pointers to our heap allocate/free functions
	state->canard = canardInit(&canardAllocate, &canardFree);
	state->canard.user_reference = state;

	//The node ID in UAVCAN is 7 bits, 0-127. Default value is CANARD_NODE_ID_UNSET (255) for anonymous mode.
	state->canard.node_id = (nodeId > CANARD_NODE_ID_MAX) ? CANARD_NODE_ID_UNSET : nodeId;

	//Sets the number of data bytes per hardware level transfer frame, 8 bytes for classic CAN, 64 bytes for CAN FD
	state->canard.mtu_bytes = CANARD_MTU_CAN_CLASSIC;

	state->getMonotonicMicroseconds = getMicroseconds;

	state->startedAt = getMicroseconds();

	return 0;
}

// Transmit pending frames from the prioritized TX queue managed by libcanard.
void canardProcessTx(CanardState* const state)
{
	const CanardFrame* frame = canardTxPeek(&state->canard);  // Take the highest-priority frame from TX queue.
	uint64_t us = state->getMonotonicMicroseconds();
	while (frame != NULL)
	{
		// Attempt transmission only if the frame is not yet timed out while waiting in the TX queue.
		// Otherwise just drop it and move on to the next one.
		if ((frame->timestamp_usec == 0) || (frame->timestamp_usec > us))
		{
			const bool result = bxCANPush(
					0,
					us,
					frame->timestamp_usec,
					frame->extended_can_id,
					frame->payload_size,
					frame->payload);
			if (result == false)
			{
				break;  //TODO: Handle errors properly
			}

		}
		//Remove the frame from the transmission queue and free its memory.
		canardTxPop(&state->canard);
		state->canard.memory_free(&state->canard, (void*) frame);

		//Check if there are any more frames to transmit.
		frame = canardTxPeek(&state->canard);
	}
}

// Process received frames by feeding them from bxCAN driver to libcanard.
// This function will invoke the "process received" handler specified during init.
void canardProcessRx(CanardState* const state)
{
	CanardFrame frame                  = {0};
	uint8_t     buffer[CANARD_MTU_CAN_CLASSIC] = {0};
	frame.payload = &buffer;
	for (uint16_t i = 0; i < FRAMES_PER_ITER; ++i)
	{
		const bool result = bxCANPop(
				0,
				&(frame.extended_can_id),
				&(frame.payload_size),
				buffer);
		if (result == false)  // The read operation has timed out with no frames, nothing to do here.
		{
			break;
		}
		// The bxCAN driver doesn't give a timestamp
		frame.timestamp_usec = state->getMonotonicMicroseconds();

		CanardTransfer transfer      = {0};
		CanardRxSubscription *subscription = NULL;
		const int8_t   canard_result = canardRxAccept2(&state->canard, &frame, 0, &transfer, &subscription);
		if (canard_result > 0)
		{
			assert(subscription != NULL);
			//If a handler was assigned as user reference, call it and pass the transfer.
			if(subscription->user_reference!=NULL)
			{
				//subscription->user_reference(state, &transfer);
			}
			else
			{
				state->transferCallback(state, &transfer);
			}

			state->canard.memory_free(&state->canard, (void*) transfer.payload);
		}
		else if ((canard_result == 0) || (canard_result == -CANARD_ERROR_OUT_OF_MEMORY))
		{
			;  // Zero means that the frame did not complete a transfer so there is nothing to do.
			// OOM should never occur if the heap is sized correctly. We track OOM errors via heap API.
		}
		else
		{
			assert(false);  // No other error can possibly occur at runtime.
		}
	}
}

/*
 * @brief 	Publishes a Heartbeat message to the CAN bus, as per the UAVCAN specification.
 * 			The local node must have a set node ID, anonymous nodes shall not publish heartbeat.
 * 			Should be published at one second intervals, a node which has not published a heartbeat
 * 			for three seconds can be considered inactive.
 *
 * @param 	CanardState	The struct containing the instance of libcanard.
 *
 * @param	currentTime	Time in microseconds from power on to present, used to calculate uptime and set timeout for transmission.
 *
 * @retval 	None
 */
void PublishHeartbeat(CanardState* const state, const CanardMicrosecond currentTime)
{
	const bool anonymous = state->canard.node_id > CANARD_NODE_ID_MAX;
	// Publish heartbeat every second unless the local node is anonymous. Anonymous nodes shall not publish heartbeat.
	if (!anonymous)
	{
		uavcan_node_Heartbeat_1_0 heartbeat = {0};
		heartbeat.uptime                    = (uint32_t) ((currentTime - state->startedAt) / 1000000);
		heartbeat.mode.value                = uavcan_node_Mode_1_0_OPERATIONAL;

		const O1HeapDiagnostics heap_diag   = o1heapGetDiagnostics(state->heap);
		if (heap_diag.oom_count > 0)
		{
			heartbeat.health.value = uavcan_node_Health_1_0_CAUTION;
		}
		else
		{
			heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
		}


		uint8_t      serialized[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
		size_t       serialized_size                                                        = sizeof(serialized);
		const int8_t err = uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, &serialized[0], &serialized_size);
		assert(err >= 0);
		if (err >= 0)
		{
			const CanardTransfer transfer = {
					.timestamp_usec = currentTime + 1000000,  // Set transmission deadline 1 second, optimal for heartbeat.
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindMessage,
					.port_id        = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
					.remote_node_id = CANARD_NODE_ID_UNSET,
					.transfer_id    = (CanardTransferID) (state->uavcan_node_heartbeat_transfer_id++),
					.payload_size   = serialized_size,
					.payload        = &serialized[0],
			};
			(void) canardTxPush(&state->canard, &transfer);
		}
	}
}
