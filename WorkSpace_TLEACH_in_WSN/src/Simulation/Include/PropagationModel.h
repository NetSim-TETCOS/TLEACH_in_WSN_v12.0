#pragma once
/************************************************************************************
* Copyright (C) 2016                                                               *
* TETCOS, Bangalore. India                                                         *
*                                                                                  *
* Tetcos owns the intellectual property rights in the Product and its content.     *
* The copying, redistribution, reselling or publication of any or all of the       *
* Product or its content without express prior written consent of Tetcos is        *
* prohibited. Ownership and / or any other right relating to the software and all  *
* intellectual property rights therein shall remain at all times with Tetcos.      *
*                                                                                  *
* Author:    Shashi Kant Suman                                                     *
*                                                                                  *
* ---------------------------------------------------------------------------------*/

#ifndef _NETSIM_PROPAGATION_MODEL_H_
#define _NETSIM_PROPAGATION_MODEL_H_
#ifdef  __cplusplus
extern "C" {
#endif

#define NEGATIVE_DBM	-1000.0
#define ZERO_MW			1.0e-100
#define MW_TO_DBM(mw)	(mw <= ZERO_MW?NEGATIVE_DBM:10.0*log10(mw))
#define DBM_TO_MW(dbm)	(dbm == NEGATIVE_DBM? 0.0: pow(10.0,dbm/10.0))

#ifndef MAC_LAYER_PROTOCOL
#define MAC_LAYER_PROTOCOL int
#endif
#ifndef __CALLBACK__
#define __CALLBACK__ 
#endif

#define PROPAGATION_PROPAGATION_MEDIUM_DEFAULT			_strdup("AIR")
#define PROPAGATION_CHANNEL_CHARACTERISTICS_DEFAULT		_strdup("FADING_AND_SHADOWING")
#define PROPAGATION_PATHLOSS_MODEL_DEFAULT				_strdup("Friis_Free_Space_propagation")
#define PROPAGATION_FADING_MODEL_DEFAULT				_strdup("NONE")
#define PROPAGATION_SHADOWING_MODEL_DEFAULT				_strdup("NONE")
#define PROPAGATION_STANDARD_DEVIATION_DEFAULT			5.0
#define PROPAGATION_PATHLOSS_EXPONENT_DEFAULT			2.0
#define PROPAGATION_FADING_FIGURE_DEFAULT				1.0
#define PROPAGATION_SHAPE_PARAMETER_DEFAULT				2.0
#define PROPAGATION_OMEGA_DEFAULT						2.0
#define PROPAGATION_THORP_SPREAD_COEFF_DEFAULT			2.0
#define PROPAGATION_TEMPERATUREZONE_COUNT_DEFAULT		0
#define PROPAGATION_Z_DEFAULT							0.0
#define PROPAGATION_TEMPERATURE_DEFAULT					25.0	// in degree celcius
#define PROPAGATION_SALINITY_DEFAULT					35
#define PROPAGATION_WINDSPEED_DEFAULT					0		// In m/s
#define PROPAGATION_SHIPPING_DEFAULT					0.5		// [0-1]

	//Typedef's
	typedef struct stru_propagation_info PROPAGATION_INFO, *PPROPAGATION_INFO;
	typedef struct stru_recvpower RECV_POWER, *PRECV_POWER;
	
	typedef bool (*fnCheckInterface)(NETSIM_ID nTxId,
									NETSIM_ID nTxInterface,
									NETSIM_ID nRxId,
									NETSIM_ID nRxInterface);
	typedef void(*fnUserPathlossModel)(PPROPAGATION_INFO pinfo,
					PRECV_POWER p);

	typedef enum
	{
		PROPAGATIONMODEL_NO_PATHLOSS,
		PROPAGATIONMODEL_PATHLOSS_ONLY,
		PROPAGATIONMODEL_PATHLOSS_AND_SHADOWING,
		PROPAGATIONMODEL_PATHLOSS_AND_FADING_AND_SHADOWING,
	}PROPAGATION_MODEL;

	typedef enum
	{
		PATHLOSSMODEL_None,
		PATHLOSSMODEL_Log_Distance,
		PATHLOSSMODEL_Friis_Free_Space,
		PATHLOSSMODEL_Cost_231_Hata_Urban,
		PATHLOSSMODEL_Cost_231_Hata_SubUrban,
		PATHLOSSMODEL_Hata_Urban,
		PATHLOSSMODEL_Hata_SubUrban,
		PATHLOSSMODEL_Indoor_Office,
		PATHLOSSMODEL_Indoor_Home,
		PATHLOSSMODEL_Indoor_Factory,
		PATHLOSSMODEL_THORP,
	}PATHLOSS_MODEL;

	typedef enum
	{
		FADINGMODEL_NONE,
		FADINGMODEL_RICIAN,
		FADINGMODEL_RAYLEIGH,
		FADINGMODEL_NAKAGAMI,
	}FADING_MODEL;

	typedef enum
	{
		SHADOWMODEL_NONE,
		SHADOWMODEL_CONSTANT,
		SHADOWMODEL_LOGNORMAL,
	}SHADOW_MODEL;

	typedef enum
	{
		PROPMEDIUM_AIR,
		PROPMEDIUM_ACOUSTICS,
	}PROP_MEDIUM;

	typedef struct stru_pathloss_var
	{
		double pathLossExponent;
		double d0;

		//Throp Progation model
		double spreadCoeff;
	}PATHLOSS_VAR,*PPATHLOSS_VAR;

	typedef struct stru_fading_var
	{
		//Nakagami parameter
		double shape_parameter; // Range: 0.5-9, default 2
		double omega; // Range: 0.25-27, default 2

	}FADING_VAR,*PFADING_VAR;

	typedef struct stru_shadowloss_var
	{
		double standardDeviation;
		bool iSet;
		double Gset;
	}SHADOW_VAR,*PSHADOW_VAR;

	typedef struct stru_temperature_zone
	{
		double lDepth;
		double hDepth;
		double avgDepth;
		double depth;
		double temperature;		// in degree celsius
	}TEMPERATUREZONE, *ptrTEMPERATUREZONE;

	typedef struct stru_acoutics_propagation_var
	{
		double dWindSpeed;		// In m/s
		double dShipping;		// [0,1]
		double dSalinity;
		UINT temperatureZoneCount;
		ptrTEMPERATUREZONE tempratureZone;
	}ACOUTICSPROPVAR, *ptrACOUTICSPROPVAR;

	typedef struct stru_propagation_model
	{
		PROP_MEDIUM propMedium;

		PROPAGATION_MODEL model;
		
		PATHLOSS_MODEL pathloss_model;
		PATHLOSS_VAR pathlossVar;

		FADING_MODEL fading_model;
		FADING_VAR fadingVar;

		SHADOW_MODEL shadow_model;
		SHADOW_VAR shadowVar;

		ptrACOUTICSPROPVAR acouticsPropVar;
	}PROPAGATION,*PPROPAGATION;

	struct stru_recvpower
	{
		double time;
		double dRxPower_mw;		// without fading loss
		double dRxPower_dbm;	// without fading loss

		double dPathloss_db;
		double dShadowLoss_db;
		struct stru_recvpower* next;
	};

	typedef struct stru_tx_info
	{
		double dCentralFrequency; // In mHz
		double dTxPower_mw;
		double dTxPower_dbm;

		double dTxGain;		// In dBi
		double dRxGain;		// In dBi
		double dTx_Rx_Distance;	// In m
		double dTxAntennaHeight; // In m
		double dRxAntennaHeight; //In m

		double d0; // In m
	}TX_INFO,*PTX_INFO;

	struct stru_propagation_info
	{
		NETSIM_ID nTxId;
		NETSIM_ID nTxInterface;
		NETSIM_ID nRxId;
		NETSIM_ID nRxInterface;

		TX_INFO txInfo;

		PRECV_POWER recvPower;
		
		PROPAGATION propagation;

		fnCheckInterface fnpCheckInterface; /* Function pointer to check whether 
											 * two device is interfering with each other or not
											 */
		fnUserPathlossModel fnpUserPathlossModel;

	};

	typedef void* PROPAGATION_HANDLE;
	typedef void(*fnGetTxInfo)(NETSIM_ID nTxId,
							   NETSIM_ID nTxInterface,
							   NETSIM_ID nRxId,
							   NETSIM_ID nRxInterface,
							   PTX_INFO Txinfo);
	typedef bool(*check_protocol_configure)(NETSIM_ID d,
											NETSIM_ID i);


	//Initialize the propagation module
	_declspec(dllexport) PROPAGATION_HANDLE propagation_init(MAC_LAYER_PROTOCOL protocol,
															 __CALLBACK__ check_protocol_configure check,
															 __CALLBACK__ fnGetTxInfo fnpGetTxInfo,
															 __CALLBACK__ fnCheckInterface checkInterference);
	
	//Free the propagation module
	_declspec(dllexport) void propagation_finish(PROPAGATION_HANDLE handle);

	//Calculate the received power due to path loss and shadow
	_declspec(dllexport) void propagation_calculate_received_power(PROPAGATION_HANDLE handle,
																   NETSIM_ID txid,
																   NETSIM_ID txif,
																   NETSIM_ID rxid,
																   NETSIM_ID rxif,
																   double time);

	//Calculate the log normal shadow loss
	_declspec(dllexport) void propagation_calculate_lognormalshadow(PPROPAGATION_INFO pinfo,
											   PRECV_POWER p);

	//Calculate the log distance path loss
	_declspec(dllexport) void propagation_calculate_logdistancepathloss(PPROPAGATION_INFO pinfo,
												   PRECV_POWER p);

	//Calculate cost 231 hata path loss. Common for urban and suburban with only cm value changes
	_declspec(dllexport) void propagation_calculate_cost231hata(PPROPAGATION_INFO pinfo,
																PRECV_POWER p,
																double cm);

	//Calculate hata suburban path loss.
	_declspec(dllexport) void propagation_calculate_hata_Suburban(PPROPAGATION_INFO pinfo,
																  PRECV_POWER p);
	//Calculate hata urban path loss.
	_declspec(dllexport) void propagation_calculate_hata_urban(PPROPAGATION_INFO pinfo,
															   PRECV_POWER p);
	//Calculate the deterministic shadow loss
	_declspec(dllexport) void propagation_calculate_deterministicshadow(PPROPAGATION_INFO pinfo,
																		PRECV_POWER p);

	//Calculate the Thorp path loss
	_declspec(dllexport) void propagation_calculate_thorpPathLoss(PPROPAGATION_INFO pinfo,
																  PRECV_POWER p);

	// Set the User pathloss model
	_declspec(dllexport) void propagation_set_user_pathloss_model(PROPAGATION_HANDLE handle,
																  __CALLBACK__ fnUserPathlossModel model);

	_declspec(dllexport) PPROPAGATION_INFO get_propagation_info(PROPAGATION_HANDLE handle,
																NETSIM_ID txid,
																NETSIM_ID txif,
																NETSIM_ID rxid,
																NETSIM_ID rxif);
	_declspec(dllexport) PTX_INFO get_tx_info(PROPAGATION_HANDLE handle,
											  NETSIM_ID txid,
											  NETSIM_ID txif,
											  NETSIM_ID rxid,
											  NETSIM_ID rxif);

	_declspec(dllexport) double propagation_get_received_power_dbm(PROPAGATION_HANDLE handle,
																   NETSIM_ID txid,
																   NETSIM_ID txif,
																   NETSIM_ID rxid,
																   NETSIM_ID rxif,
																   double time);

	_declspec(dllexport) double propagation_calculate_fadingloss(PROPAGATION_HANDLE handle,
																 NETSIM_ID txid,
																 NETSIM_ID txif,
																 NETSIM_ID rxid,
																 NETSIM_ID rxif);

	//Propagation delay
	_declspec(dllexport) double propagation_air_calculate_propagationDelay(NETSIM_ID tx,
																		   NETSIM_ID rx,
																		   PPROPAGATION propagation);

	_declspec(dllexport) double propagation_acoutics_calculate_propagationDelay(NETSIM_ID tx,
																				NETSIM_ID rx,
																				PPROPAGATION propagation);

	//Noise
	_declspec(dllexport) double propagation_acoutics_calculate_noise(double frequency_kHz,
																	 double shipping,
																	 double windSpeed);

#ifdef  __cplusplus
}
#endif
#endif //_NETSIM_PROPAGATION_MODEL_H_
