#pragma once

#include <CoreMinimal.h>
#include <Engine/GameInstance.h>
#include <Engine/EngineTypes.h>
#include <Runtime/Launch/Resources/Version.h>
#include "ROSIntegrationCore.h"

#include "ROSIntegrationGameInstance.generated.h"

// Lets the game instance share with any bound delegates that the ROS connection status has changed
DECLARE_MULTICAST_DELEGATE_OneParam(FOnROSConnectionStatus, bool /*IsConnected*/);

UCLASS()
class ROSINTEGRATION_API UROSIntegrationGameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	virtual void Init() override;
	virtual void Shutdown() override;
	virtual void BeginDestroy() override;

public:
	UPROPERTY()
	UROSIntegrationCore* ROSIntegrationCore = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	FString ROSBridgeServerProtocol = "tcp";

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	FString ROSBridgeServerHost = "127.0.0.1";

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	int32 ROSBridgeServerPort = 9090;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS")
	bool bConnectToROS = true;

	UPROPERTY(BlueprintReadOnly, Category = "ROS")
	bool bIsConnected = false;

	UPROPERTY(EditAnywhere, Category = "ROS")
	bool bSimulateTime = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	bool bUseFixedUpdateInterval = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS", Meta = (EditCondition = "bUseFixedUpdateInterval"))
	float FixedUpdateInterval = 0.01666666667;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS")
	bool bCheckHealth = true;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS", Meta = (EditCondition = "bCheckHealth"))
	float CheckHealthInterval = 1.0f;

	FOnROSConnectionStatus OnROSConnectionStatus;

protected:
	void CheckROSBridgeHealth();

	void ShutdownAllROSObjects();

	void MarkAllROSObjectsAsDisconnected();

#if ENGINE_MINOR_VERSION > 23 || ENGINE_MAJOR_VERSION >4
	virtual void OnWorldTickStart(UWorld * World, ELevelTick TickType, float DeltaTime);
#else 
	virtual void OnWorldTickStart(ELevelTick TickType, float DeltaTime);
#endif

	FTimerHandle TimerHandle_CheckHealth;
	bool bTimerSet = false;  // has the time been set?

	bool bReconnect = false;

	FCriticalSection initMutex_;

	UPROPERTY()
	class UTopic* ClockTopic = nullptr;

	bool bAddedOnWorldTickDelegate = false;
};


template<class T>
class FLockGuard
{
	T* lockable_;
public:
	FLockGuard(T* lockable) : lockable_(lockable) { lockable->Lock(); }
	~FLockGuard() { lockable_->Unlock(); }
};
typedef FLockGuard<FCriticalSection> FLocker;

