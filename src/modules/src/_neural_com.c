// #include "config.h"
// #include "debug.h"
// #include "FreeRTOS.h"
// #include "queue.h"
// #include "static_mem.h"
// #include "task.h"
// #include "crtp.h"
// #include "neural_com.h"
// #include "neural_control.h"
// #include <stdio.h>
// #include <string.h>
// #include "logRingBuffer.h"

// static bool isInit = false;

// const float16_t outputScalerPWM[2][4] = {
//     {30000.0, 30000.0, 30000.0, 30000.0},
//     {30000.0, 30000.0, 30000.0, 30000.0}};

// const float16_t outputScalerAttRate[2][4] = {
//     {30000.0, 60, 60, 60},
//     {30000.0, 0.0, 0.0, 0.0}};

// const float16_t outputScalerAtt[2][4] = {
//     {45000.0, 10.0, 10.0, 10.0},
//     {10000.0, 0.0, 0.0, 0.0}};

// static controlState_t controlState;
// static CRTPPacket p;
// static size_t networksStored = 0;
// static Network *activeNetwork;

// static void neuralComTask(void *);
// STATIC_MEM_TASK_ALLOC(neuralComTask, NEURALCOM_TASK_STACKSIZE);


// void neuralComTaskInit()
// {
//   STATIC_MEM_TASK_CREATE(neuralComTask, neuralComTask, NEURALCOM_TASK_NAME, NULL, NEURALCOM_TASK_PRI);
//   activeNetwork = malloc(sizeof(*activeNetwork));
//   isInit = true;
// }

// bool neuralComTaskTest()
// {
//   return isInit;
// }

// static void neuralComTask(void *parameters)
// {
//   DEBUG_PRINT("neuralCom task main function is running!\n");
//   crtpInitTaskQueue(CRTP_PORT_NEURALCOM);
//   neuralControlTaskEnqueueNetwork(activeNetwork);

//   controlState = CONTROL_STATE_IDLE;
//   neuralControlTaskEnqueueControlState(controlState);

//   while (1)
//   {
//     handlePacketReceiving();
//   }
// }

// void handlePacketReceiving()
// {
//   if(crtpReceivePacketWait(CRTP_PORT_NEURALCOM, &p, 5) == pdTRUE)
//   {
//     neuralControlTaskPeekControlState(&controlState);
//     switch (*(p.data))
//     {
//     case COMMAND_TRANSFER_NETWORK:
//       // Receive network only if the drone is not being controlled by the network
//       if (controlState != CONTROL_STATE_NEURAL)
//       {
//         DEBUG_PRINT("Begin network transfer...\n");
//         if (networksStored > 0)
//         {
//           DEBUG_PRINT("Deleting old network...\n");
//           networkDestroy(*activeNetwork);
//           networksStored--;
//         }

//         // Allocate memory for the new network,
//         Network newNetwork = malloc(sizeof(*newNetwork));

//         // receive its contents,
//         neuralComReceiveNetwork(newNetwork);

//         // and set it as the new active network
//         *activeNetwork = newNetwork;

//         networksStored++;

//         DEBUG_PRINT("Network transfer complete!\n");
//       }
//       else
//       {
//         DEBUG_PRINT("Can not receive network since drone is still in neural control mode!\n");
//       }

//       break;
//     case COMMAND_START_NEURAL_CONTROL:
//       if (networksStored != 0)
//       {
//         DEBUG_PRINT("Switching to neural control!\n");
//         controlState = CONTROL_STATE_NEURAL;
//         neuralControlTaskEnqueueControlState(controlState);

//         // Setup logging queue and wait for data:
//         sendLogs();
//       }
//       else
//       {
//         DEBUG_PRINT("No network stored!\n");
//       }

//       break;
//     case COMMAND_STOP_NEURAL_CONTROL:
//       DEBUG_PRINT("Stopping network control!\n");
//       controlState = CONTROL_STATE_HOVER;
//       neuralControlTaskEnqueueControlState(controlState);
//       break;

//     case COMMAND_HOVER:
//       DEBUG_PRINT("Going into hover mode!\n");
//       controlState = CONTROL_STATE_HOVER;
//       neuralControlTaskEnqueueControlState(controlState);

//       break;
//     case COMMAND_LAND:
//       DEBUG_PRINT("Landing...\n");
//       controlState = CONTROL_STATE_LAND;
//       neuralControlTaskEnqueueControlState(controlState);

//       break;
//     default:
//       DEBUG_PRINT("Unknown Command!\n");
//     }
//     vTaskDelay(5);
//   }
// }

// void neuralComReceiveNetwork(Network network)
// {
//   recvChecksum(network);
//   recvStructure(network);
//   recvScaling(network);
//   recvActivation(network);
//   recvWeights(network);
//   recvCtrlMode(network);
//   checkChecksum(network);
// }

// void recvStructure(Network network)
// {
//   CRTPPacket p;
//   size_t networkSize = 0;
//   size_t layerSize[7];

//   crtpReceivePacketBlock(CRTP_PORT_NEURALCOM, &p);
  
//   while (p.data[networkSize] > 0)
//   {
//     layerSize[networkSize] = p.data[networkSize];
//     networkSize++;
//   }
  
//   networkInit(network, networkSize, layerSize);
// }

// void recvScaling(Network network)
// {
//   CRTPPacket p;
//   float16_t *floatArray;
//   size_t scaler_idx = 0;
//   size_t neuron_idx = 0;
//   float16_t scaler0[network->layers[0].layerSize];
//   float16_t scaler1[network->layers[0].layerSize];
//   int done = 0;
//   while (!done)
//   {
//     crtpReceivePacketBlock(CRTP_PORT_NEURALCOM, &p);
//     floatArray = (float16_t *)p.data;
//     for (int i = 0; i < 14; i++)
//     {
//       if (scaler_idx==0)
//       {
//         scaler0[neuron_idx] = *(floatArray + i);
//        }
//       else
//       {
//         scaler1[neuron_idx] = *(floatArray + i);
//       }
//       neuron_idx++;
//       if (neuron_idx == network->layers[0].layerSize)
//       {
//         neuron_idx = 0;
//         scaler_idx++;
//         if (scaler_idx == 2)
//         {
//           done = 1;
//           break;
//         }
//       }
//     }  
//   }
//   DEBUG_PRINT("DEBUG: input layer size %d \n", (int) network->layers[0].layerSize);
//   networkSetInputNormalizer(network, scaler0, scaler1);
// }
// void recvActivation(Network network)
// {
//   CRTPPacket p;
//   crtpReceivePacketBlock(CRTP_PORT_NEURALCOM, &p);
//   activationFunction_t activation = p.data[0];
//   for (size_t layer = 1; layer < network->networkSize-1; layer++)
//   {
//     networkSetActivationFunction(network, layer, activation);
//   }
//   networkSetActivationFunction(network, network->networkSize-1, IDENTITY);
// }

// void recvWeights(Network network)
// {
//   CRTPPacket p;
//   float16_t *floatArray;
//   size_t layer_idx = 1;
//   size_t neuron_idx = 0;
//   size_t weight_idx = 0;
//   int done = 0;
//   while (!done)
//   {
//     crtpReceivePacketBlock(CRTP_PORT_NEURALCOM, &p);
//     floatArray = (float16_t *)p.data;
//     for (int i = 0; i < 14; i++)
//     {
//       if (weight_idx == network->layers[layer_idx - 1].layerSize)
//       {
//         networkSetBias(network, layer_idx, neuron_idx, *(floatArray + i));
//        }
//       else
//       {
//         networkSetWeight(network, layer_idx, neuron_idx, weight_idx, *(floatArray + i));
//       }
//       weight_idx++;
//       if (weight_idx > network->layers[layer_idx - 1].layerSize)
//       {
//         weight_idx = 0;
//         neuron_idx++;
//         if (neuron_idx == network->layers[layer_idx].layerSize)
//         {
//           DEBUG_PRINT("INFO: Initialized %d neuros in Layer %d \n", neuron_idx, layer_idx);
//           neuron_idx = 0;
//           layer_idx++;
//         }
//         if (layer_idx == network->networkSize)
//         {
//           done = 1;
//           break;
//         }
//       }
//     }
//   }
// }

// int recvCtrlMode(Network network)
// {
//   CRTPPacket p;
//   crtpReceivePacketBlock(CRTP_PORT_NEURALCOM, &p);
//   controlMode_t controlMode = p.data[0];
//   float16_t *outputScaler;
//   switch (controlMode)
//   {
//   case CONTROL_MODE_MOTORS:
//     outputScaler = (float16_t *)outputScalerPWM;
//     break;
//   case CONTROL_MODE_ATTITUDE_RATE:
//     outputScaler = (float16_t *)outputScalerAttRate;
//     break;
//   case CONTROL_MODE_ATTITUDE:
//     outputScaler = (float16_t *)outputScalerAtt;
//     break;
//   default:
//     DEBUG_PRINT("Invalid control mode!");
//     return 1;
//   }
//   networkSetOutputScaler(network, outputScaler, outputScaler + 4);
//   networkSetControlMode(network, controlMode);
//   return 0;
// }

// void neuralControlTaskEnqueueLogs(float16_t log)
// {
//   if(!logRB_in(&log)){
//       // Log Overflow -> stop neural control
//       DEBUG_PRINT("log-overflow-");
//       neuralControlTaskEnqueueControlState(CONTROL_STATE_HOVER);
//   };
// }

// void sendLogs()
// {
//   float16_t logPacket[14];
//   float16_t log;
//   int log_counter = 0;
//   CRTPPacket pa;
//   pa.header=CRTP_HEADER(CRTP_PORT_NEURALLOG, 0);
//   pa.size = 30;

//   // send logs in 14 float16s per packet until no new logs in queue for 100ms
//   while(logRB_out(&log,100))
//   {
//     //DEBUG_PRINT(" %f", (double) log);
//     logPacket[log_counter] = log;
//     log_counter ++;
    
//     if(log_counter == 14)
//     {
//       log_counter = 0;
//       memcpy(pa.data, logPacket, 14*2);
//       crtpSendPacket(&pa);
//       DEBUG_PRINT(".");
//       handlePacketReceiving();
//     } 
   
//   }

//   // send emtpy package as end message:
//   for(int i=0; i<14; i++){
//     logPacket[i] = (float16_t) 0.0;
//   }
//   memcpy(pa.data, logPacket, 14*2);
//   vTaskDelay(100);
//   crtpSendPacket(&pa);

//   DEBUG_PRINT("\nall logs sent!\n");

// }

// void recvChecksum(Network network){
//   CRTPPacket p;
//   crtpReceivePacketBlock(CRTP_PORT_NEURALCOM, &p);
//   float16_t checksum = ((float16_t *)p.data)[0];
//   DEBUG_PRINT("Checksum: %f \n",(double) checksum);
//   network->checksum = checksum;
// }
// void checkChecksum(Network network){
//   float16_t calculated_checksum = networkCalculateChecksum(network);
//   float16_t checksum = network->checksum;

//   DEBUG_PRINT("Checksum Comparison: %f vs %f \n",(double)checksum,(double)calculated_checksum);
  
// }
