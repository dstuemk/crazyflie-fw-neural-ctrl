// #include "neural_net.h"
// #include "debug.h"
// #include <math.h>

// netErrCode_t networkInit(Network network, size_t networkSize, size_t *layerSize)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     // Allocate memory for the layers
//     network->layers = malloc(sizeof(*(network->layers)) * networkSize);
//     network->networkSize = networkSize;

//     //Save the number of neurons in previous layer to know how many weights we need in the next
//     size_t prevLayerSize = 0;

//     layer_t * layer = network->layers;
//     for (size_t l = 0; l < network->networkSize; l++)
//     {
//         // Allocate memory for the layers
//         layer->neurons = malloc(sizeof(*(layer->neurons)) * *layerSize);
//         layer->layerSize = *layerSize;

//         neuron_t * neuron = layer->neurons;
//         for (size_t n = 0; n < layer->layerSize; n++)
//         {
//             // Allocate memory for the weights
//             neuron->weights = malloc(sizeof(*(neuron->weights)) * prevLayerSize);
//             neuron->activation = 0.0;
//             neuron->bias = 0.0;

//             neuron++;
//         }

//         prevLayerSize = *layerSize;
//         layer++;
//         layerSize++;
//     }

//     return ret;
// }

// netErrCode_t networkDestroy(Network network)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     layer_t * layer = network->layers;

//     for (size_t l = 0; l < network->networkSize; l++)
//     {
//         neuron_t * neuron = layer->neurons;
//         for (size_t n = 0; n < layer->layerSize; n++)
//         {
//             free(neuron->weights);
//             neuron++;
//         }
//         free(layer->neurons);
//         layer++;
//     }
//     free(network->layers);
//     return ret;
// }

// netErrCode_t networkSetActivationFunction(Network network, size_t layer, activationFunction_t activationFunction)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     float16_t (*activationFunctionHandle)(float16_t);

//     switch (activationFunction)
//     {
//     case IDENTITY:
//         activationFunctionHandle = activationFunctionIdentity;
//         break;
//     case RELU:
//         activationFunctionHandle = activationFunctionRelu;
//         break;
//     case TANH:
//         activationFunctionHandle = activationFunction_tanh;
//         break;
//     case SVIDENTITY:
//         activationFunctionHandle = activationFunctionSvidentity;
//         break;
//     default:
//         DEBUG_PRINT("ERROR: Unknown activation function %d in Layer %d!",activationFunction, layer);
//         return UNKNOWN_ACTIVATION_FUNCTION;
//     }

//     network->layers[layer].activationFunction = activationFunctionHandle;

//     return ret;
// }

// netErrCode_t networkSetWeight(Network network, size_t layer, size_t neuron, size_t weight, float16_t value)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     network->layers[layer].neurons[neuron].weights[weight] = value;

//     return ret;
// }

// netErrCode_t networkSetBias(Network network, size_t layer, size_t neuron, float16_t value)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     network->layers[layer].neurons[neuron].bias = value;

//     return ret;
// }

// netErrCode_t networkSetInputNormalizer(Network network, float16_t * a, float16_t * b)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     layer_t * inputLayer = network->layers; // inputLayer = layers[0]
//     DEBUG_PRINT("INFO: input layer size: %d \n", (int) inputLayer->layerSize);
//     network->inputNormalizer.a = malloc(inputLayer->layerSize * sizeof(float16_t));
//     network->inputNormalizer.b = malloc(inputLayer->layerSize * sizeof(float16_t));

//     for (size_t n = 0; n < inputLayer->layerSize; n++)
//     {
//         network->inputNormalizer.a[n] = *a++;
//         network->inputNormalizer.b[n] = *b++;
//     }

//     return ret;
// }

// netErrCode_t networkSetOutputScaler(Network network, float16_t * a, float16_t * b)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     layer_t * outputLayer = network->layers + (network->networkSize - 1);

//     network->outputScaler.a = malloc(outputLayer->layerSize * sizeof(float16_t));
//     network->outputScaler.b = malloc(outputLayer->layerSize * sizeof(float16_t));

//     for (size_t n = 0; n < outputLayer->layerSize; n++)
//     {
//         network->outputScaler.a[n] = *a++;
//         network->outputScaler.b[n] = *b++;
//     }

//     return ret;
// }

// netErrCode_t networkSetControlMode(Network network, controlMode_t controlMode)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     network->controlMode = controlMode;

//     return ret;
// }

// netErrCode_t networkNormalizeInput(Network network, float16_t *input, float16_t *normalizedInput)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     float16_t *a = network->inputNormalizer.a;
//     float16_t *b = network->inputNormalizer.b;

//     for (size_t n = 0; n < network->layers[0].layerSize; n++)
//     {
//         (*normalizedInput) = ((*input) - (*a)) / (*b); // (x-mean) / sigma
//         input++;
//         normalizedInput++;
//         a++;
//         b++;
//     }

//     return ret;
// }

// netErrCode_t networkFeedForward(Network network, float16_t *input, float16_t *output)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     layer_t * layer = network->layers;

//     // Copy inputs into the activations of the first layer (index 0)
//     neuron_t * neuron = layer->neurons;
//     for (size_t n = 0; n < layer->layerSize; n++)
//     {
//         neuron->activation = (*input);
//         neuron++;
//         input++;
//     }

//     // Save the previous layer for easy access to its neurons activations
//     layer_t * prevLayer = layer;
//     layer++;

//     neuron_t * prevNeuron;
//     float16_t *weight;

//     // Start the loop with the first hidden layer
//     for (size_t l = 1; l < network->networkSize; l++)
//     {
//         neuron = layer->neurons;
//         for (size_t n = 0; n < layer->layerSize; n++)
//         {
//             // Initialize the neurons activation with its bias
//             neuron->activation = neuron->bias;

//             prevNeuron = prevLayer->neurons;
//             weight = neuron->weights;

//             for (size_t prev_n = 0; prev_n < prevLayer->layerSize; prev_n++)
//             {
//                 // DEBUG_PRINT("Layer: %d\tNeuron: %d\tWeight: %f\n", l, n, (double) *weight);
//                 // add the activation of the previous neurons to this neurons activation
//                 // multiplied by the appropriate weight
//                 neuron->activation += (*weight) * prevNeuron->activation;
//                 weight++;
//                 prevNeuron++;
//             }

//             // feed the activation value into the layers activation function
//             neuron->activation = layer->activationFunction(neuron->activation);
//             neuron++;
//         }

//         prevLayer = layer;
//         layer++;
//     }

//     // Write the activation of the last layers neurons to the ouput
//     layer_t * lastLayer = network->layers + network->networkSize - 1;
//     neuron = lastLayer->neurons;
//     for (size_t n = 0; n < lastLayer->layerSize; n++)
//     {
//         (*output) = neuron->activation;
//         output++;
//         neuron++;
//     }

//     return ret;
// }

// netErrCode_t networkScaleOutput(Network network, float16_t *output)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     float16_t *a = network->outputScaler.a;
//     float16_t *b = network->outputScaler.b;

//     for (size_t n = 0; n < network->layers[network->networkSize - 1].layerSize; n++)
//     {
//         (*output) = (*a) * (*output) + (*b); // a*x + b
//         output++;
//         a++;
//         b++;
//     }

//     return ret;
// }

// netErrCode_t networkClipOutput(Network network, float16_t *input, float16_t *output)
// {
//     netErrCode_t ret = NET_SUCCESS;

//     for (size_t n = 0; n < network->layers[network->networkSize - 1].layerSize; n++)
//     {
//         (*output) = activationFunctionSvidentity(*input);
//         output++;
//         input++;
//     }

//     return ret;
// }

// float16_t activationFunctionIdentity(float16_t x)
// {
//     return x;
// }

// float16_t activationFunctionRelu(float16_t x)
// {
//     if (x < 0)
//     {
//         return 0;
//     }
//     else
//     {
//         return x;
//     }
// }

// float16_t activationFunction_tanh(float16_t x)
// {
//     return (float16_t) tanhf((float)x);
// }

// float16_t activationFunctionSvidentity(float16_t x)
// {
//     return (x < -1) ? -1 : (x > 1) ? 1 : x;
// }

// float16_t networkCalculateChecksum(Network network){

//     float16_t input[network->layers[0].layerSize];
//     float16_t output[4];

//     for(int i=0; i<network->layers[0].layerSize; i++){
//         input[i] = 1;
//     }
//     networkFeedForward(network, input, output);

//     return output[0]+output[1]+output[2]+output[3];
// }
