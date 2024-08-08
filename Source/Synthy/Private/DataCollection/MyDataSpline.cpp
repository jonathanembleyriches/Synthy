// Fill out your copyright notice in the Description page of Project Settings.


#include "DataCollection/MyDataSpline.h"

// Sets default values
AMyDataSpline::AMyDataSpline()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	m_Spline = CreateDefaultSubobject<USplineComponent>(TEXT("Spline"));
}

// Called when the game starts or when spawned
void AMyDataSpline::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AMyDataSpline::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

