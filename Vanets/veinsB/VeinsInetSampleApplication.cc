//
// Copyright (C) 2018 Christoph Sommer <sommer@ccs-labs.org>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins_inet/VeinsInetSampleApplication.h"

#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/TagBase_m.h"
#include "inet/common/TimeTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include <iostream>
#include <string>


#include "veins_inet/VeinsInetSampleMessage_m.h"

using namespace std;

using namespace inet;

Define_Module(VeinsInetSampleApplication);

VeinsInetSampleApplication::VeinsInetSampleApplication()
{
}

const char* msgr[9];
bool haveForwarded = false;
int msgRec=0;
double lastC=48; //41 40.5 40.2 sim3 32 sim2 21
//E2 empieza en 47
vector<int> env(30,0);
    
vector<int> renv(30,0);

vector<vector<std::string>> pastVal(31, vector<std::string>(6, ""));

vector<vector<const char *>> nodo(31,vector<const char *>);


bool VeinsInetSampleApplication::startApplication()
{
    

    // host[0] should stop at t=20s
/*
    //Sim1
    if (getParentModule()->getIndex() == 0) {
        auto callback = [this]() {
            getParentModule()->getDisplayString().setTagArg("i", 1, "red");

            traciVehicle->setSpeed(0);

            auto payload = makeShared<VeinsInetSampleMessage>();
            payload->setChunkLength(B(100));
            payload->setRoadId(traciVehicle->getRoadId().c_str());
            timestampPayload(payload);

            auto packet = createPacket("Accident");
            packet->insertAtBack(payload);
            sendPacket(std::move(packet));

            // host should continue after 30s
            auto callback = [this]() {
                traciVehicle->setSpeed(-1);
            };
            timerManager.create(veins::TimerSpecification(callback).oneshotIn(SimTime(74000, SIMTIME_MS)));
        };
        int cont=70150;
        timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(70150, SIMTIME_MS)));//while, el callback espera a la senal
    }
*/



    //sim3
    //cambio de via, los coches desaceleran
    //un coche se incorpora
/*
    if (getParentModule()->getIndex() == 6) {
            auto callback = [this]() {
                getParentModule()->getDisplayString().setTagArg("i", 1, "red");

                //traciVehicle->changeRoute("E31", 200000);
                traciVehicle->setSpeed(2);

                auto payload = makeShared<VeinsInetSampleMessage>();
                payload->setChunkLength(B(100));
                payload->setRoadId(traciVehicle->getRoadId().c_str());
                timestampPayload(payload);

                auto packet = createPacket("ChangingLane");
                packet->insertAtBack(payload);
                sendPacket(std::move(packet));

                // host should continue after 30s
                auto callback = [this]() {
                    traciVehicle->setSpeed(-1);
                };
                timerManager.create(veins::TimerSpecification(callback).oneshotIn(SimTime(185000, SIMTIME_MS)));
            };
            timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(165000, SIMTIME_MS)));//while, el callback espera a la senal
       }
*/

    //SIM4
    srand((unsigned) time(NULL));
        double cont=0;
        int cont2=0;
        int updateTimet=1000; //200 500 y 1000
        int cont3=1;
        //int startTimet=14000;
        //if (getParentModule()->getIndex() != 1 and getParentModule()->getIndex() != 6 and getParentModule()->getIndex() != 9) {
                auto callback = [this]() {
                    getParentModule()->getDisplayString().setTagArg("i", 1, "red");
                    //Los vehiculos deben estar en la simulacion
                    //otro caso un vehiculo cambia de carril, el otro desacelera
                    //traciVehicle->setSpeed(0);

                    auto payload = makeShared<VeinsInetSampleMessage>();
                    payload->setChunkLength(B(100));/////
                    //payload->setChunkLength(B(rand()%100));
                    //payload->setRoadId(traciVehicle->getRoadId().c_str());
                    int a=getParentModule()->getIndex();
                    payload->setRoadId(("E"+ std::to_string(a)).c_str());

                    //payload->setRoadId((getParentModule()->getIndex().c_str()+cont2.c_str()).c_str());
                    timestampPayload(payload);

                    auto packet = createPacket("detectionsUpdate");
                    packet->insertAtBack(payload);
                    sendPacket(std::move(packet));

                    env[getParentModule()->getIndex()]=env[getParentModule()->getIndex()]+1;
                    EV_INFO << "Envia " << getParentModule()->getIndex()<< " num "<<env[getParentModule()->getIndex()]<<endl;
                   

                    //EV_INFO << "Envia " << getParentModule()->getIndex()<< " num "<<cont3;
                    //cont3++;
                  };
                //int random = rand() % 100;
                //cont=startTimett+random*10;
                //simtime_t a=24000;
                //cont=24000;
                //Probar frec, elegir un tiempo maximo simulacion 120

                while((47000+cont*updateTimet)<=59000){
                    timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(47000+cont*updateTimet+rand()%1000, SIMTIME_MS))); //en % cambiar a 200, 500 y 1000
                    cont=cont+1;
                }


                //EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<cont;

//sim 1 40000-80000     //E2 47000/87000
            //sim2 21000 - 36000
//SIM3 32000-56000


               //Sim E2 con 20 nodos para 100 metros
                //E2 empieza en 47000 hasta 59000
            //random = rand();
            //cont=startTimet+updateTimet+random*100;
            //timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(cont, SIMTIME_MS)));
            //random = rand();
            //cont=startTimet+updateTimet*2+random*100;
            //timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(cont, SIMTIME_MS)));



    return true;
}

bool VeinsInetSampleApplication::stopApplication()
{
    return true;
}

VeinsInetSampleApplication::~VeinsInetSampleApplication()
{
}

void VeinsInetSampleApplication::processPacket(std::shared_ptr<inet::Packet> pk)
{
/*
    //sim1
    auto payload = pk->peekAtFront<VeinsInetSampleMessage>();

    EV_INFO << "Received packet: " << payload << endl;
    //EV_DETAIL << "Content: " << payload->getRoadId().c_str() << endl;

    getParentModule()->getDisplayString().setTagArg("i", 1, "green");

    //traciVehicle->changeRoute(payload->getRoadId(), 999.9);
    const char* aux = "E18";
        const char* aux2 = "E14";
        const char* aux3 = "E16";

        //if(getParentModule()->getIndex()==4){
        //Los vehiculos en la seccion de la ruta 29 desaceleran
        if(strcmp(traciVehicle->getRoadId().c_str(), aux)==0 or strcmp(traciVehicle->getRoadId().c_str(), aux2)==0 or strcmp(traciVehicle->getRoadId().c_str(), aux3)==0){
            traciVehicle->setSpeed(1);
        }

    if (haveForwarded) return;

    auto packet = createPacket("relay");
    packet->insertAtBack(payload);
    sendPacket(std::move(packet));

    haveForwarded = true;
    //getParentModule()->getDisplayString().setTagArg("i", 1, "gray");
*/
        auto payload = pk->peekAtFront<VeinsInetSampleMessage>();
        simtime_t a=simTime();
        EV_INFO << "Received packet0: " << payload<< " ruta "<<payload->getRoadId() << " tiempo "<<a<<" lastC "<<lastC<<endl;
        //EV_INFO << "Received packet2: " << payload<< " ruta "<<payload->getLength() << " vs "<< aux <<endl;
        //EV_DETAIL << "Content: " << payload->getRoadId().c_str() << endl;

        getParentModule()->getDisplayString().setTagArg("i", 1, "green");

        if(std::find(nodo[getParentModule()->getIndex()].begin(), nodo[getParentModule()->getIndex()].end(), payload->getRoadId()) != nodo[getParentModule()->getIndex()].end()){
            haveForwarded = true;
        }
        else{
            haveForwarded = false;
        }

        /*
        if (strcmp(payload->getRoadId(), aux) == 0){
        //if (strcmp(payload->getChunkLength(), aux) == 0){
            haveForwarded = true;
        }
        else{
            if (strcmp(payload->getRoadId(), aux2) == 0){
                //if (strcmp(payload->getChunkLength(), aux) == 0){
                    haveForwarded = true;
            }
            else{
                if (strcmp(payload->getRoadId(), aux3) == 0){
                    //if (strcmp(payload->getChunkLength(), aux) == 0){
                        haveForwarded = true;
                    }
                else{
                    if (strcmp(payload->getRoadId(), aux4) == 0){
                                    //if (strcmp(payload->getChunkLength(), aux) == 0){
                                        haveForwarded = true;
                                    }
                                else{
                                    if (strcmp(payload->getRoadId(), aux5) == 0){
                                                                        //if (strcmp(payload->getChunkLength(), aux) == 0){
                                                                            haveForwarded = true;
                                    }
                                    else{
                                        if (strcmp(payload->getRoadId(), aux6) == 0){
                                                                                                                //if (strcmp(payload->getChunkLength(), aux) == 0){
                                                                                                                    haveForwarded = true;
                                                                            }
                                        else{
                                            haveForwarded = false;

                                        }
                                    }
                                }

                }
            }
        }
        */

        //traciVehicle->changeRoute(payload->getRoadId(), 999.9);

        if (haveForwarded){
            //return;
        }
        else{
            auto packet = createPacket("relay");
            packet->insertAtBack(payload);
            sendPacket(std::move(packet));
            //EV_INFO << "reenvia: " << getParentModule()->getIndex()<< " num  "<<contR;
            //contR++;
            //haveForwarded = true;
            //msgr[getParentModule()->getIndex()]=payload->getRoadId();
            
            renv[getParentModule()->getIndex()]=renv[getParentModule()->getIndex()]+1;
            EV_INFO << "reenvia: " << getParentModule()->getIndex()<< " num "<<renv[getParentModule()->getIndex()]<<endl;
            nodo[getParentModule()->getIndex()].push_back(payload->getRoadId());
            nodo[getParentModule()->getIndex()].push_back("NO");
        }
}
