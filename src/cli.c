/****************************************************************************
 * nxp_bms/BMS_v1/src/cli.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <arch/board/board.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <ctype.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "data.h"
#include "cli.h"

#include <nuttx/vt100.h>

/****************************************************************************
 * Defines
 ****************************************************************************/
#define DEFAULT_PRIORITY 	100
#define DEFAULT_STACK_SIZE 	1024

#define HELP_COMMAND 		"help" 
#define GET_COMMAND 		"get"
#define SET_COMMAND 		"set"
#define SHOW_COMMAND 		"show"

#define RESET_COMMAND		"reset"
#define SLEEP_COMMAND 		"sleep"
#define WAKE_COMMAND 		"wake"
#define DEEP_SLEEP_COMMAND  "deepsleep"
#define SAVE_COMMAND		"save"
#define LOAD_COMMAND 		"load"
#define DEFAULT_COMMAND		"default"

#define AMOUNT_COMMANDS 	11

#define PARAMS_COMMAND 		"parameters"
#define SHOW_MEAS_COMMAND	"show-meas"

#define SHOW_STACK_VOLTAGE 	"v-batt"
#define SHOW_BAT_VOLTAGE	"v-out"
#define SHOW_CELL_VOLTAGE	"v-cell"
#define SHOW_OUTPUT_STATUS	"s-out"
#define SHOW_CURRENT		"i-batt"
#define SHOW_TEMPERATURE	"c-bms"
#define SHOW_AVG_CURRENT	"i-avg"
#define SHOW_AVG_POWER		"p-avg"
#define SHOW_ENGERGY_COMS	"e-used"
#define SHOW_STATE_O_CHARGE "s-charge"
#define SHOW_REMAINING_CAP	"a-rem"
#define SHOW_ALL 			"all"
#define SHOW_TOP 			"top"

#define PARAMETER_ARRAY_SIZE NONE + 2

// because a measured update sequence (bms show top 1 and bms show all 1) takes 42ms, max wait time will be 100ms
#define CLI_TIMED_LOCK_WAIT_TIME_MS 100
#define MS_TO_NS_MULT				1000000
#ifndef NSEC_MAX 
#define NSEC_MAX 					999999999
#endif

/****************************************************************************
 * Types
 ****************************************************************************/
//typedef enum {HELP = 0, GET = 1, SET = 2, SHOW = 3, WRONG = 4}commands_t;

/****************************************************************************
 * private data
 ****************************************************************************/

// the string array for the states
static const char *gStatesArray[] = 
{
	//"INIT", "NORMAL", "CHARGE", "SLEEP", "OCV", "FAULT", "SELF_DISCHARGE", "DEEP_SLEEP"
	FOR_EACH_STATE(GENERATE_STRING)
};
// the string array for the states
static const char *gChargeStatesArray[] = 
{
	//"INIT", "NORMAL", "CHARGE", "SLEEP", "OCV", "FAULT", "SELF_DISCHARGE", "DEEP_SLEEP"
	FOR_EACH_CHARGE_STATE(GENERATE_STRING)
};


// the string array for the parameters
static const char *gGetSetParameters[PARAMETER_ARRAY_SIZE] = {

	// generate a string for all the parameters in the FOR_EACH_PARAMETER define in BMS_data_types.h
	FOR_EACH_PARAMETER(GENERATE_STRING)

	// add these 2 strings to it
	"STATE",
	"ALL"				
};

//! this array consists of all the non writable parameters
const parameterKind_t nonWritableParameters[] =
{
	C_BATT,
	V_OUT,
	V_BATT,
	I_BATT,
	I_BATT_AVG,
	S_OUT,
	P_AVG,
	E_USED,
	T_FULL,
	S_FLAGS,
	S_HEALTH,
	S_CHARGE,
	V_CELL1,
	V_CELL2,
	V_CELL3,
	V_CELL4,
	V_CELL5,
	V_CELL6,
	C_AFE,
	C_T,
	C_R,
	NONE 
};

//! these are the datatypes for each parameter in the right sequence
const char *parametersTypes[] = 
{
  "float",
  "float",
  "float",
  "float",
  "float",
  "bool",
  "float",
  "float",
  "float",
  "float",
  "float",
  "uint8",
  "uint8",
  "uint8",
  "uint8",
  "int32",
  "char*",
  "float",
  "float",
  "float",
  "float",
  "float",
  "float",
  "float",
  "float",
  "float",
  "uint16",
  "uint16",
  "uint8",
  "uint16",
  "uint16",
  "uint8",
  "uint8",
  "float",
  "float",
  "float",
  "float",
  "float",
  "float",
  "float",
  "uint16",
  "uint8",
  "uint8",
  "uint8",
  "uint16",
  "uint16",
  "float",
  "float",
  "uint8",
  "int32",
  "int32",
  "float",
  "float",
  "float",
  "float",
  "uint8",
  "bool",
  "bool",
  "uint8",
  "uint16",
  "uint8",
  "int32",
  "int32",
  "uint8",
  "uint8",
  "uint16",
  "uint8",
  "uint16",
  "uint8",
  "uint8"
};

//states_t *gStateAdr;
//pthread_mutex_t *gStateMutexAdr;
bool gCliInitialized = false;

bool gCliPrintLockInitialized = false;
static pthread_mutex_t gCliPrintLock;		

static char gGetSetParamsLowerString[32];

getMainStateCallbackBatFuntion 		gGetMainStateCallbackBatFuntionfp;
getChargeStateCallbackBatFuntion 	gGetChargeStateCallbackBatFuntionfp;
userCommandCallbackBatFuntion 		gUserCommandCallbackBatFuntionfp;
/****************************************************************************
 * private Functions
 ****************************************************************************/
//! to print the help 
void printHelp(void);

//! this prints all the parameters from gGetSetParameters
void printParameters(void);

//! this prints all the parameters from FOR_EACH_PARAMETER with the value
void printAllParameterValues(void);

/****************************************************************************
 * public Functions
 ****************************************************************************/
/*!
 * @brief 	this function is needed to get the address of the state and the mutex to the cli
 * 			
 * @param 	p_getChargeStateCallbackBatFuntion the address of the function to get the main state
 * @param 	p_userCommandCallbackBatFuntion the address of function to get the charge state
 * @param 	p_userCommandCallbackBatFuntion address of the function to be called when a commands needs to be processed by the main
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 */
int cli_initialize(getMainStateCallbackBatFuntion p_getMainStateCallbackBatFuntion, 
	getChargeStateCallbackBatFuntion p_getChargeStateCallbackBatFuntion, 
	userCommandCallbackBatFuntion p_userCommandCallbackBatFuntion)
{
	int lvRetValue = 0;

	// check if initialized
	if(!gCliInitialized)
	{
		// initialize the mutex
		pthread_mutex_init(&gCliPrintLock, NULL);
		gCliPrintLockInitialized = true;

		// connect the callback functions
		gGetMainStateCallbackBatFuntionfp = p_getMainStateCallbackBatFuntion;
		gGetChargeStateCallbackBatFuntionfp = p_getChargeStateCallbackBatFuntion;
		gUserCommandCallbackBatFuntionfp = p_userCommandCallbackBatFuntion;


		// set that it is initialized
		gCliInitialized = true;

	}

	// return
	return lvRetValue;
}

/*!
 * @brief 	this function is deinitialize the cli
 * 			
 * @param 	None
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example if(cli_deinitialize())
 *			{
 *				// do something with the error
 *			}
 */
int cli_deinitialize(void)
{
	// reset the initialized variable
	gCliInitialized = false;

	// return
	return 0;
}

/****************************************************************************
 * main
 ****************************************************************************/
/*!
 * @brief 	this function is the cli function. 
 * 			it will take care of the commands send to the BMS
 * 			like the "help", "set" and "get" command
 * 			
 * @param 	the amount of arguments in char *argv[] 
 * @param 	the arguments to the function
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 * @example (in main.c) cli_processCommands(argc, argv);
 */
int cli_processCommands(int argc, char **argv)
{
	commands_t lvCommands = CLI_WRONG; 
	showCommands_t lvShowCommands = CLI_NONE;

	//cli_printf("Hello, World! test!");
	FAR char *lvCommandString = argv[1];
	FAR char *lvParameterString = argv[2];
	FAR char *lvValueString = argv[3];

	int lvRetValue = -1;
	bool lvFoundParam = false;
	bool lvSendParameters = false;
	bool lvSendShowCommands = false;
	bool lvGetAll = false;
	bool nonWritableFound = false;
	int i, j;
	const char *lvCommandArray[AMOUNT_COMMANDS] = {HELP_COMMAND, GET_COMMAND, SET_COMMAND, SHOW_COMMAND,
												   RESET_COMMAND, SLEEP_COMMAND, WAKE_COMMAND, DEEP_SLEEP_COMMAND,
												   SAVE_COMMAND, LOAD_COMMAND, DEFAULT_COMMAND};

	const char *lvShowCommandArgArr[] = {SHOW_STACK_VOLTAGE, SHOW_BAT_VOLTAGE, SHOW_CELL_VOLTAGE, SHOW_OUTPUT_STATUS,
										 SHOW_CURRENT, SHOW_TEMPERATURE, SHOW_AVG_CURRENT, SHOW_AVG_POWER, 
										 SHOW_ENGERGY_COMS, SHOW_STATE_O_CHARGE, SHOW_REMAINING_CAP, SHOW_ALL, SHOW_TOP};
	parameterKind_t lvParameter = NONE;
	float lvFloatVal;
	char *lvPStringVal;
	int32_t lvIntVal = 0;

	// check if initialized
	if(!gCliInitialized)
	{
		cli_printf("CLI isn't initialized, please initialize cli\n");
		return lvRetValue;
	}

 	// check which command it is
 	switch(argc)
 	{
 		// only one command
 		case 2:
 			// check for a help command
 			if((!strncmp(lvCommandString, lvCommandArray[HELP_INDEX], strlen(lvCommandArray[HELP_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_HELP;
 			}
 			else if((!strncmp(lvCommandString, lvCommandArray[RESET_INDEX], strlen(lvCommandArray[RESET_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_RESET;
 			}
 			else if((!strncmp(lvCommandString, lvCommandArray[SLEEP_INDEX], strlen(lvCommandArray[SLEEP_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_SLEEP;
 			}
 			else if((!strncmp(lvCommandString, lvCommandArray[WAKE_INDEX], strlen(lvCommandArray[WAKE_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_WAKE;
 			}
 			else if((!strncmp(lvCommandString, lvCommandArray[DEEP_SLEEP_INDEX], strlen(lvCommandArray[DEEP_SLEEP_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_DEEP_SLEEP;
 			}
 			else if((!strncmp(lvCommandString, lvCommandArray[SAVE_INDEX], strlen(lvCommandArray[SAVE_INDEX]))))
 			{
 				// set the command 
 				lvCommands = CLI_SAVE;
 			}
 			else if((!strncmp(lvCommandString, lvCommandArray[LOAD_INDEX], strlen(lvCommandArray[LOAD_INDEX]))))
 			{
 				// set the command 
 				lvCommands = CLI_LOAD;
 			}
 			else if((!strncmp(lvCommandString, lvCommandArray[DEFAULT_INDEX], strlen(lvCommandArray[DEFAULT_INDEX]))))
 			{
 				// set the command 
 				lvCommands = CLI_DEFAULT;
 			}

 		break;

 		// two commands
 		case 3:
 			// check for a get command
 			if((!strncmp(lvCommandString, lvCommandArray[GET_INDEX], strlen(lvCommandArray[GET_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_GET;
 			}

 			// check for help parameters command
 			else if((!strncmp(lvCommandString, lvCommandArray[HELP_INDEX], strlen(lvCommandArray[HELP_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_HELP;

 				if((!strcmp(lvParameterString, PARAMS_COMMAND)))//, strlen(PARAMS_COMMAND))))
 				{
 					// set it to true
 					lvSendParameters = true;
 				}
 				else if ((!strcmp(lvParameterString, SHOW_MEAS_COMMAND)))//, strlen(SHOW_MEAS_COMMAND))))
 				{
 					// set the variable
 					lvSendShowCommands = true;
 				}
 				else
 				{
 					// if wrong third command 
 					// reset the commands
 					lvCommands = CLI_WRONG; 
 				}
 			}

 			break;

 		// three commands 
 		case 4:
 			// check for a set command
 			if((!strncmp(lvCommandString, lvCommandArray[SET_INDEX], strlen(lvCommandArray[SET_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_SET;
 			}

 			// check for the show command
 			else if((!strncmp(lvCommandString, lvCommandArray[SHOW_INDEX], strlen(lvCommandArray[SHOW_INDEX]))))
 			{
 				// set the command
 				lvCommands = CLI_SHOW;
 			}
 		break;

 		// too little or too much commands
 		default:
 			// there is an error
			cli_printf("Wrong input!\ttry \"bms help\"\n");
			return lvRetValue;
 		break;
 	}

 	// do somthing with the different commands
 	switch(lvCommands)
 	{
 		// in case of a wrong input (should've already be filtered out)
 		case CLI_WRONG: 
 			// there is an error
			cli_printf("Wrong input!\ttry \"bms help\"\n");
			break;
		// in case the user wants the help
		case CLI_HELP:
			// print the help
			if(lvSendParameters)
			{
				printParameters();
			}
			else if(lvSendShowCommands)
			{
				// print the show commands
				for(i = 0; i < CLI_NONE; i++)
				{
					// print them
					cli_printf("%s \n", lvShowCommandArgArr[i]);
				}
			}
			else
			{
				printHelp();
			}

			lvRetValue = 0;
			break;

		// in case the user wants to get or set a parameter
		case CLI_SET:
		case CLI_GET:

			lvRetValue = 0;
			// go through the string array
			for (i = 0; i < (PARAMETER_ARRAY_SIZE); i++)
			{
				// change the get set parameter to lowercase and convert all underscores to upderscores
				// change each character of the string parameters to lowercase for the user input
				for(j = 0; (gGetSetParameters[i])[j]; j++)
				{
					// change it to lowercase
				 	gGetSetParamsLowerString[j] = tolower((gGetSetParameters[i])[j]);

				 	// convert all underscores to upderscores
				 	// find the underscores
					if(((gGetSetParameters[i])[j]) == '_')
					{
						// change to upperscore
						gGetSetParamsLowerString[j] = '-';
					}
				}

				// add the null terminator to the string
				gGetSetParamsLowerString[j] = '\0';

				// compare the string
				if((strncmp(lvParameterString, gGetSetParamsLowerString, strlen(gGetSetParameters[i])) == 0) 
					&& (strlen(lvParameterString) == strlen(gGetSetParameters[i])))
				{
					// check if the parameter can be set
					if(i <= NONE)
					{
						// save the parameter
						lvParameter = (parameterKind_t)(i);
						lvFoundParam = true;
						break;
					}
					
					// it is ALL
					// check if it is GET
					if(lvCommands == CLI_GET)
					{
						// set the get all parameter true
						lvGetAll = true;

						// set the first parameter
						lvParameter = 0;
						lvFoundParam = true;
						break;
					}				
				}
				else if(i > NONE)
				{
					// there is an error
					cli_printf("Wrong parameter input! \ttry \"bms help\"\n"); 
				}
			}

			// check if get or set
			if(lvCommands == CLI_GET && lvFoundParam)
			{
				if(lvGetAll)
				{
					// call get all function
					printAllParameterValues();
					break;
				}

				// say the standard message
				cli_printf("%s = ", lvParameterString);

				// get the parameter to the user
				if(lvParameter != NONE)
				{
					// get the parameter from the data sturct
					switch(data_getType(lvParameter))
					{
						// if it is a floating point
						case FLOATVAL:
							// get the value
							if(data_getParameter(lvParameter, &lvFloatVal, NULL) == NULL)
								// something went wrong
								cli_printf("Error CLI: data_getParameter\n");
							else
								cli_printf("%.3f", lvFloatVal);
						break;
						// if it is a string
						case STRINGVAL:
							// get the value
							lvPStringVal = data_getParameter(lvParameter, NULL, NULL);
							if(lvPStringVal == NULL)
								// something went wrong
								cli_printf("Error CLI: data_getParameter\n");
							else
								cli_printf("%s", lvPStringVal);
						break;
						// if it is a integer (max int32_t)
						default:
							// get the value
							if(data_getParameter(lvParameter, &lvIntVal, NULL) == NULL)
								// something went wrong
								cli_printf("Error CLI: data_getParameter\n");
							else
								cli_printf("%d", lvIntVal);
						break;
					}

					// check if there is a unit
					if(*data_getUnit(lvParameter) != '-')
					{
						// add the unit 
						cli_printf(" %s\n", data_getUnit(lvParameter));
					}
					else
					{
						// add the next line
						cli_printf("\n");
					}
				}
				else
				// the user wants the state	
				{
					// check if the charge state needs to be outputted as well
					if(gGetMainStateCallbackBatFuntionfp() == CHARGE)
					{
						cli_printf("\"%s-%s\"\n", gStatesArray[(int)(gGetMainStateCallbackBatFuntionfp())], 
							gChargeStatesArray[(int)(gGetChargeStateCallbackBatFuntionfp())]);
					}
					else
					{
						// output the state
						cli_printf("\"%s\"\n", gStatesArray[(int)(gGetMainStateCallbackBatFuntionfp())]);
					}
				}
			}
			else if (lvCommands == CLI_SET && lvFoundParam)
			{
				// set the parameter to the user
				if(lvParameter != NONE)
				{
					// check if it is the parameters that are able to be set
					for(i = 0; nonWritableParameters[i] != NONE; i++)
					{
						// check if it is non writable
						if(lvParameter == nonWritableParameters[i])
						{
							// this should be written
							// set the value
							nonWritableFound = true;

							// break the for loop
							break;
						}
					}

					// check if the value may be written
					if(!nonWritableFound)
					{
						// TODO do we need to add a password?
						//cli_printf("add password!\n");
						// check which type the value should be
						// get the parameter from the data sturct
						switch(data_getType(lvParameter))
						{
							// if it is a floating point
							case FLOATVAL:
								// get the value

								// convert string to float
								lvFloatVal = strtof(lvValueString, NULL);

								// inform user
								cli_printf("setting %s with \"%.3f\"...\n", lvParameterString, lvFloatVal);

								// check for errors
								if(lvFloatVal == 0 && (errno == ERANGE || errno == EINVAL))
								{
									// inform user and return
									cli_printf("Error conversion!\n");
									lvRetValue = -1;
	
									return lvRetValue;
								}

								//set the parameter and check if failed
								if(data_setParameter(lvParameter, &lvFloatVal))
								{
									// if failed
									cli_printf("Failed!\tMaybe outside minimum or maximum\n");
								}
								else
								{ 	
									// if succeeded
									cli_printf("succeeded!\n");
								}

							break;
							// if it is a string
							case STRINGVAL:

								// inform user
								cli_printf("setting %s with \"%s\"...\n", lvParameterString, lvValueString);

								// check if the string lenght is OK
								if(strlen(lvValueString) > MODEL_NAME_MAX_CHARS)
								{
									// it is too long
									cli_printf("input string too long! max characters is %d\n", MODEL_NAME_MAX_CHARS);
									lvRetValue = -1;
									break;
								}

								// set the parameter and check if failed
								if(data_setParameter(lvParameter, lvValueString))
								{	
									//  if failed
									cli_printf("Failed!\n");
								}
								else
								{	
									// if succeeded
									cli_printf("succeeded!\n");
								}

							break;
							// if it is a integer (max int32_t)
							default:
								// get the value

								// convert string to float
								// watch out this returns a long int!
								lvIntVal = strtol(lvValueString, NULL, 10);

								// inform user
								cli_printf("setting %s with \"%d\"...\n", lvParameterString, lvIntVal);

								// check for errors
								if(lvIntVal == 0 && (errno == ERANGE || errno == EINVAL))
								{
									// inform user and return
									cli_printf("Error conversion!\n");
									lvRetValue = -1;	

									return lvRetValue;
								}

								// set the parameter and check if failed
								if(data_setParameter(lvParameter, &lvIntVal))
								{	
									//  if failed
									cli_printf("Failed!\tMaybe outside minimum or maximum\n");
								}
								else
								{	
									// if succeeded
									cli_printf("succeeded!\n");
								}
							break;
						}
					}
					// if it is one of the parameters that shouldn't be written
					else
					{
						cli_printf("%s may not be written!\n", lvParameterString);
					}
				}
				// the user wants to set the state (command to go somewhere)
				else
				{
					cli_printf("%s may not be written!\n", lvParameterString);
				}
			}

		break;

		// in case of show
		case CLI_SHOW:

			// check the second parameters
			for(i = 0; i < CLI_NONE; i++)
			{
				// check each 
				if(!strncmp(lvParameterString, lvShowCommandArgArr[i], strlen(lvShowCommandArgArr[i])))
				{
					// found it 
					lvShowCommands = (showCommands_t)i;
					break;
				}
			}

			// if succeeded
			if(lvShowCommands != CLI_NONE)
			{
				// check the value
				lvIntVal = strtol(lvValueString, NULL, 10);
				if(lvIntVal == 1 || lvIntVal == 0)
				{
					// call the callback
					gUserCommandCallbackBatFuntionfp(lvCommands, lvShowCommands, lvIntVal);

					// output to user
					if(lvIntVal)
					{
						cli_printf("enabled visability for %s\n", lvShowCommandArgArr[lvShowCommands]);
					}
					else
					{
						cli_printf("disabled visability for %s\n", lvShowCommandArgArr[lvShowCommands]);
					}
				}
				else
				{
					cli_printf("wrong value! try \"bms help\"\n");
				}
			}
			else
			{
				cli_printf("wrong show command! try \"bms help show-meas\"\n");
			}
			
		// it is an other command
		default:
			// call the callback
			gUserCommandCallbackBatFuntionfp(lvCommands, lvShowCommands, lvIntVal);
		break;
 	}

	// return
  	return lvRetValue;
}

/*!
 * @brief 	this function is the same as cli_printf(), but it will make sure it can be used
 * 			by multiple threads 
 * 			
 * @param 	fmt This is the string that contains the text to be written to stdout. 
 * 			It can optionally contain embedded format tags that are replaced by 
 * 			the values specified in subsequent additional arguments and formatted as requested. 
 * 			Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param 	the arguments of the function
 *
 * @return 	If successful, the total number of characters written is returned. 
 *			On failure, a negative number is returned.
 */
int cli_printf(FAR const IPTR char *fmt, ...)
{
	va_list ap;
  	int     lvRetValue;
  	struct timespec waitTime;

  	// check if mutex is initialzed 
  	if(gCliPrintLockInitialized)
  	{
  		// get the time
		if(clock_gettime(CLOCK_REALTIME, &waitTime) == -1)
		{
			cli_printf("cli ERROR: failed to get time!\n");
		}

		// add the time 
		waitTime.tv_sec 	+= 	(int)(CLI_TIMED_LOCK_WAIT_TIME_MS / 1000) + 
								((waitTime.tv_nsec + (CLI_TIMED_LOCK_WAIT_TIME_MS % 1000) * MS_TO_NS_MULT) / (NSEC_MAX+1)); 
  	 	waitTime.tv_nsec 	= 	(waitTime.tv_nsec + (CLI_TIMED_LOCK_WAIT_TIME_MS % 1000) * MS_TO_NS_MULT) % (NSEC_MAX+1) ; 

  		// lock the mutex
		lvRetValue = pthread_mutex_timedlock(&gCliPrintLock, &waitTime);

		// check if succesfull
		if(!lvRetValue)
		{
	  		// initialze variable list ap
		 	va_start(ap, fmt);

		 	// do the printf
			lvRetValue = vprintf(fmt, ap);

			// end the variable list
			va_end(ap);

			// unlock the mutex
			pthread_mutex_unlock(&gCliPrintLock);
  		}
  		else
  		{
  			// initialze variable list ap
		 	va_start(ap, fmt);

		 	// do the printf
			lvRetValue = vprintf(fmt, ap);

			// end the variable list
			va_end(ap);
  		}
	}

  	else
  	{
  		// initialze variable list ap
	 	va_start(ap, fmt);

	 	// do the printf
		lvRetValue = vprintf(fmt, ap);

		// end the variable list
		va_end(ap);
  	}

  	// return to the user
	return lvRetValue;
}

/*!
 * @brief 	this function is the same as cli_printf(), but with no mutex lock
 * @warning should be used with cli_printLock()
 * 			
 * @param 	fmt This is the string that contains the text to be written to stdout. 
 * 			It can optionally contain embedded format tags that are replaced by 
 * 			the values specified in subsequent additional arguments and formatted as requested. 
 * 			Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param 	the arguments of the function
 *
 * @return 	If successful, the total number of characters written is returned. 
 *			On failure, a negative number is returned.
 */
int cli_printfNoLock(FAR const IPTR char *fmt, ...)
{
	va_list ap;
  	int     lvRetValue; 

	// initialze variable list ap
 	va_start(ap, fmt);

 	// do the printf
	lvRetValue = vprintf(fmt, ap);

	// end the variable list
	va_end(ap);

  	// return to the user
	return lvRetValue;
}

/*!
 * @brief 	this function is the same as cli_printf(), but with a try mutex lock
 * @warning should be used with cli_printLock()
 * 			
 * @param 	fmt This is the string that contains the text to be written to stdout. 
 * 			It can optionally contain embedded format tags that are replaced by 
 * 			the values specified in subsequent additional arguments and formatted as requested. 
 * 			Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param 	the arguments of the function
 *
 * @return 	If successful, the total number of characters written is returned. 
 *			On failure, a negative number is returned.
 */
int cli_printfTryLock(FAR const IPTR char *fmt, ...)
{
	va_list ap;
  	int     lvRetValue; 
  	int 	mutexState;

  	// check if mutex is initialzed 
  	if(gCliPrintLockInitialized)
  	{
  		// lock the mutex
		mutexState = pthread_mutex_trylock(&gCliPrintLock);

  		// initialze variable list ap
	 	va_start(ap, fmt);

	 	// do the printf
		lvRetValue = vprintf(fmt, ap);

		// end the variable list
		va_end(ap);

		// check if the mutex was locked 
		if(!mutexState)
		{
			// unlock the mutex
			pthread_mutex_unlock(&gCliPrintLock);
		}
  	}
  	else
  	{
  		// initialze variable list ap
	 	va_start(ap, fmt);

	 	// do the printf
		lvRetValue = vprintf(fmt, ap);

		// end the variable list
		va_end(ap);
  	}

  	// return to the user
	return lvRetValue;
}

/*!
 * @brief 	this function is used to lock and unlock the cli_print mutex. 
 * 			it can be used to make sure updating the measured data is done without 
 * 			writting random things in between.
 * 			
 * @param 	lock when true it will lock, when false it will unlock the cli_print mutex
 *
 * @return 	0 If successful, otherwise an error will indicate the error
 */
int cli_printLock(bool lock)
{
	int lvRetValue = -1; 
	// int fd;
 	// struct termios attributes;

	// check if initialzed
	if(gCliPrintLockInitialized)
	{
		// check if needed to be locked
		if(lock)
		{
			// lock the mutex
			lvRetValue = pthread_mutex_lock(&gCliPrintLock);
		}
		else
		{
			// unlock the mutex
			lvRetValue = pthread_mutex_unlock(&gCliPrintLock);
		}
	}

	return lvRetValue;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

// this prints the help
void printHelp(void)
{
	// print the help
	cli_printf("This is the bms cli (command line interface) help\n");
	cli_printf("These commands can be used with the bms:\n");
	cli_printf("bms help                \t--this command shows this help\n");
	cli_printf("bms help parameters     \t--this command shows the <parameter> list\n");
	cli_printf("bms help show-meas      \t--this command shows the <show-meas> list\n");
	cli_printf("bms get <parameter>     \t--this command gets a parameter value.\n");
	cli_printf("                        \t  parameter is the parameter you want\n");
	cli_printf("bms get all 		    \t--this command gets all the parameters\n");
	cli_printf("                        \t  including the values\n");	
	cli_printf("bms set <parameter> <x> \t--this command can be used to set a parameter\n");
	cli_printf("                        \t  WARNING this could lead to unsave operations!\n");
	cli_printf("                        \t  WARNING only use this command in a safe manner!\n");
	cli_printf("                        \t  parameter is the parameter you want to set\n");
	cli_printf("                        \t  x is the new value of the parameter you want to set\n");
	cli_printf("                        \t  to enter a decimal value use \".\" as seperator\n");
	cli_printf("                        \t  to enter a string with spaces use \"input string\"\n");
	cli_printf("bms show <show-meas> <x>\t--this command can be used to show the cyclic measurement results\n");
	cli_printf("                        \t  show-meas is the to measurement to enable or disable visibility\n");
	cli_printf("                        \t  if x is 1 the measurement is shown, if 0 it will be disabled\n");
	cli_printf("bms reset               \t--this command will reset the fault when in the fault state\n");
	cli_printf("bms sleep               \t--with this command it will go to the sleep state\n");
	cli_printf("                       	\t  from the normal or the self_discharge state\n");
	cli_printf("                        \t  NOTE: if current is drawn it will transition to the normal state\n");
	cli_printf("bms wake                \t--this command will wake the BMS in the sleep state\n");
	cli_printf("bms deepsleep           \t--with this command it will go to the deep sleep state\n");
	cli_printf("                        \t  from the sleep state or the charge state (from charge not implemented yet)\n");
	cli_printf("bms save                \t--this command will save the current settings (parameters) to flash\n");
	cli_printf("bms load                \t--this command will load the saved settings (parameters) from flash\n");
	cli_printf("bms default             \t--this command will load the default settings\n");
	cli_printf("some parameters have a letter in front of the \"-\", this indicates the type of parameter\n");
	cli_printf("a - capacity\n");
	cli_printf("c - temperature (celcius)\n");
	cli_printf("e - energy\n");
	cli_printf("i - current\n");
	cli_printf("n - number\n");
	cli_printf("s - status\n");
	cli_printf("t - time\n");
	cli_printf("v - voltage\n");
}

// this prints all the parameters from gGetSetparameters
void printParameters(void)
{
	int i, j;
	bool nonWritableFound = false;

	// standard message
	cli_printf("These are the <parameter> inputs that can be used to get or set a parameter:\n");

	cli_printf("parameter"); 

	// move the cursor to position 
	cli_printf("\r" VT100_FMT_CURSORRT, 22);

	cli_printf("unit");

	// move the cursor to position 
	cli_printf("\r" VT100_FMT_CURSORRT, 28);

	cli_printf("RO/RW");

	// move the cursor to position 
	cli_printf("\r" VT100_FMT_CURSORRT, 35);

	cli_printf("type\n");

	// loop though the parameters
	for(i = 0; i < PARAMETER_ARRAY_SIZE; i++)
	{
		// change each character of the string parameters to lowercase for the user input
		for(j = 0; (gGetSetParameters[i])[j]; j++)
		{
			// change it to lowercase
		 	gGetSetParamsLowerString[j] = tolower((gGetSetParameters[i])[j]);

		 	// find the underscores
			if(((gGetSetParameters[i])[j]) == '_')
			{
				// change to upperscore
				gGetSetParamsLowerString[j] = '-';
			}
		}

		// add the null terminator to the string
		gGetSetParamsLowerString[j] = '\0';

		// print them
		cli_printf("%s", gGetSetParamsLowerString);

		// move the cursor to position 
		cli_printf("\r" VT100_FMT_CURSORRT, 22);

		// make sure you stay in the array
		if(i < (sizeof(parametersTypes)/sizeof(char*)))
		{
			cli_printf("%s", data_getUnit(i));
		}
		else
		{
			cli_printf("-");
		}

		// move the cursor to position 
		cli_printf("\r" VT100_FMT_CURSORRT, 30);

		// reset the value
		nonWritableFound = false;

		// check if the parameter is non writable
		for(j = 0; nonWritableParameters[j] != NONE; j++)
		{
			// check if the parameter is non writable 
			if(i == nonWritableParameters[j])
			{
				// set the value and escape the for loop
				nonWritableFound = true;
				break;
			}
		}

		// check if non writable
		if(nonWritableFound || i >= NONE)
		{
			cli_printf("RO");
		}
		else
		{
			cli_printf("RW");
		}

		// move the cursor to position 
		cli_printf("\r" VT100_FMT_CURSORRT, 35);

		// make sure you stay in the array
		if(i < (sizeof(parametersTypes)/sizeof(char*)))
		{
			cli_printf("%s\n", parametersTypes[i]);
		}
		else
		{
			cli_printf("-\n");
		}
	}

	// return
	return;
}


//! this prints all the parameters from FOR_EACH_PARAMETER with the value
void printAllParameterValues(void)
{
	int i, j;
	float lvFloatVal = 0.0;
	char *lvPStringVal;
	int32_t lvIntVal;

	// standard message
	cli_printf("These are the parameters with the values:\n");

	// loop though the parameters
	for(i = 0; i < NONE; i++)
	{
		// reset the intvalue
		lvIntVal = 0;

		// change each character of the string parameters to lowercase for the user input
		// convert all underscores to upderscores
		for(j = 0; (gGetSetParameters[i])[j]; j++)
		{
			// change it to lowercase
		 	gGetSetParamsLowerString[j] = tolower((gGetSetParameters[i])[j]);

		 	// find the underscores
			if(((gGetSetParameters[i])[j]) == '_')
			{
				// change to upperscore
				gGetSetParamsLowerString[j] = '-';
			}
		}

		// add the null terminator to the string
		gGetSetParamsLowerString[j] = '\0';

		// print them
		cli_printf("%s", gGetSetParamsLowerString);//, strlen(gGetSetParamsLowerString));

		// move the cursor to position 
		cli_printf("\r" VT100_FMT_CURSORRT, 22);

		// get the parameter value from the data sturct
		switch(data_getType(i))
		{
			// if it is a floating point
			case FLOATVAL:
				// get the value
				if(data_getParameter(i, &lvFloatVal, NULL) == NULL)
					// something went wrong
					cli_printf("Error CLI: data_getParameter\n");
				else
					cli_printf("%.3f", lvFloatVal);
			break;
			// if it is a string
			case STRINGVAL:
				// get the value
				lvPStringVal = data_getParameter(i, NULL, NULL);
				if(lvPStringVal == NULL)
					// something went wrong
					cli_printf("Error CLI: data_getParameter\n");
				else
					cli_printf("%s", lvPStringVal);
			break;
			// if it is a integer (max int32_t)
			default:
				// get the value
				if(data_getParameter(i, &lvIntVal, NULL) == NULL)
					// something went wrong
					cli_printf("Error CLI: data_getParameter\n");
				else
					cli_printf("%d", lvIntVal);
			break;
		}

		// move the cursor to position 
		cli_printf("\r" VT100_FMT_CURSORRT, 33);

		// output the unit
		// make sure you stay in the array
		if(i < (sizeof(parametersTypes)/sizeof(char*)))
		{
			cli_printf("%s\n", data_getUnit(i));
		}
		else
		{
			cli_printf("-\n");
		}
	}

	// return
	return;
}
