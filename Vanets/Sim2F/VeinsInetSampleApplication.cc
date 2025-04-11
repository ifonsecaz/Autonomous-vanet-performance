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

int msgRec=0;
double lastC=40.1; //41 40.5 40.2 sim3 32 sim2 21

int env0=0;
int env1=0;
int env2=0;
int env3=0;
int env4=0;
int env5=0;
int env6=0;
int env7=0;
int env8=0;

int renv0=0;
int renv1=0;
int renv2=0;
int renv3=0;
int renv4=0;
int renv5=0;
int renv6=0;
int renv7=0;
int renv8=0;

const char* pastVal0="";
const char* pastVal1="";
const char* pastVal2="";
const char* pastVal3="";
const char* pastVal4="";
const char* pastVal5="";
const char* pastVal6="";
const char* pastVal7="";
const char* pastVal8="";
const char* pastVal9="";
const char* pastVal01="";
const char* pastVal11="";
const char* pastVal21="";
const char* pastVal31="";
const char* pastVal41="";
const char* pastVal51="";
const char* pastVal61="";
const char* pastVal71="";
const char* pastVal81="";
const char* pastVal02="";
const char* pastVal12="";
const char* pastVal22="";
const char* pastVal32="";
const char* pastVal42="";
const char* pastVal52="";
const char* pastVal62="";
const char* pastVal72="";
const char* pastVal82="";
const char* pastVal03="";
const char* pastVal13="";
const char* pastVal23="";
const char* pastVal33="";
const char* pastVal43="";
const char* pastVal53="";
const char* pastVal63="";
const char* pastVal73="";
const char* pastVal83="";

const char* pastVal04="";
const char* pastVal14="";
const char* pastVal24="";
const char* pastVal34="";
const char* pastVal44="";
const char* pastVal54="";
const char* pastVal64="";
const char* pastVal74="";
const char* pastVal84="";
const char* pastVal05="";
const char* pastVal15="";
const char* pastVal25="";
const char* pastVal35="";
const char* pastVal45="";
const char* pastVal55="";
const char* pastVal65="";
const char* pastVal75="";
const char* pastVal85="";

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
        int updateTimet=100; //200 500 y 1000
        int cont3=1;
        //int startTimet=14000;
        if (getParentModule()->getIndex() != 1 and getParentModule()->getIndex() != 6 and getParentModule()->getIndex() != 9) {
                auto callback = [this]() {
                    getParentModule()->getDisplayString().setTagArg("i", 1, "red");
                    //Los vehiculos deben estar en la simulacion
                    //otro caso un vehiculo cambia de carril, el otro desacelera
                    //traciVehicle->setSpeed(0);

                    auto payload = makeShared<VeinsInetSampleMessage>();
                    payload->setChunkLength(B(100));
                    //payload->setChunkLength(B(rand()%100));
                    //payload->setRoadId(traciVehicle->getRoadId().c_str());
                    int a=getParentModule()->getIndex();
                    payload->setRoadId(("E"+ std::to_string(a)).c_str());

                    //payload->setRoadId((getParentModule()->getIndex().c_str()+cont2.c_str()).c_str());
                    timestampPayload(payload);

                    auto packet = createPacket("detectionsUpdate");
                    packet->insertAtBack(payload);
                    sendPacket(std::move(packet));
                    if(getParentModule()->getIndex()==0){
                                           env0=env0+1;
                                           EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env0;

                                            }
                                            if(getParentModule()->getIndex()==1){
                                                env1=env1+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env1;

                                               }
                                            if(getParentModule()->getIndex()==2){
                                                env2=env2+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env2;

                                               }
                                            if(getParentModule()->getIndex()==3){
                                                env3=env3+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env3;

                                               }
                                            if(getParentModule()->getIndex()==4){
                                                env4=env4+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env4;

                                               }
                                            if(getParentModule()->getIndex()==5){
                                                env5=env5+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env5;

                                               }
                                            if(getParentModule()->getIndex()==6){
                                                env6=env6+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env6;

                                               }
                                            if(getParentModule()->getIndex()==7){
                                                env7=env7+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env7;

                                               }
                                            if(getParentModule()->getIndex()==8){
                                                env8=env8+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env8;

                                               }

                    //EV_INFO << "Envia " << getParentModule()->getIndex()<< " num "<<cont3;
                    //cont3++;
                  };
                //int random = rand() % 100;
                //cont=startTimett+random*10;
                //simtime_t a=24000;
                //cont=24000;
                //Probar frec, elegir un tiempo maximo simulacion 120

                while((40000+cont*updateTimet)<=80000){
                    timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(40000+cont*updateTimet+rand()%100, SIMTIME_MS))); //en % cambiar a 200, 500 y 1000
                    cont=cont+1;
                }


                //EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<cont;

//sim 1 40000-80000
            //sim2 21000 - 36000
//SIM3 32000-56000
            //random = rand();
            //cont=startTimet+updateTimet+random*100;
            //timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(cont, SIMTIME_MS)));
            //random = rand();
            //cont=startTimet+updateTimet*2+random*100;
            //timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(cont, SIMTIME_MS)));

    }

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


    //sim2
    const char* aux=""; //buffer
        const char* aux2="";
        const char* aux3="";
        const char* aux4="";
        const char* aux5="";
        const char* aux6="";
        int contR=1;
        //getParentModule()->getDisplayString().setTagArg("i", 1, "gray");

        auto payload = pk->peekAtFront<VeinsInetSampleMessage>();
        double t2 = simTime().dbl();
        if(lastC+0.0<t2){
            lastC=lastC+0.1;//1 0.5 0.2

            pastVal0="";
            pastVal1="";
            pastVal2="";
            pastVal3="";
            pastVal4="";
            pastVal5="";
            pastVal6="";
            pastVal7="";
            pastVal8="";
            pastVal9="";
            pastVal01="";
            pastVal11="";
            pastVal21="";
            pastVal31="";
            pastVal41="";
            pastVal51="";
            pastVal61="";
            pastVal71="";
            pastVal81="";
            pastVal02="";
            pastVal12="";
            pastVal22="";
            pastVal32="";
            pastVal42="";
            pastVal52="";
            pastVal62="";
            pastVal72="";
            pastVal82="";
            pastVal03="";
                   pastVal13="";
                   pastVal23="";
                   pastVal33="";
                   pastVal43="";
                   pastVal53="";
                   pastVal63="";
                   pastVal73="";
                   pastVal83="";
           pastVal04="";
           pastVal14="";
           pastVal24="";
           pastVal34="";
           pastVal44="";
           pastVal54="";
           pastVal64="";
           pastVal74="";
           pastVal84="";
           pastVal05="";
                       pastVal15="";
                       pastVal25="";
                       pastVal35="";
                       pastVal45="";
                       pastVal55="";
                       pastVal65="";
                       pastVal75="";
                       pastVal85="";

        }

        if(getParentModule()->getIndex()==0){
            aux=pastVal0;
            aux2=pastVal01;
            aux3=pastVal02;
            aux4=pastVal03;
            aux5=pastVal04;
            aux6=pastVal05;

        }
        if(getParentModule()->getIndex()==1){
               aux=pastVal1;
               aux2=pastVal11;
               aux3=pastVal12;
               aux4=pastVal13;
               aux5=pastVal14;
                           aux6=pastVal15;
           }
        if(getParentModule()->getIndex()==2){
               aux=pastVal2;
               aux2=pastVal21;
               aux3=pastVal22;
               aux4=pastVal23;
               aux5=pastVal24;
                           aux6=pastVal25;
           }
        if(getParentModule()->getIndex()==3){
               aux=pastVal3;
               aux2=pastVal31;
               aux3=pastVal32;
               aux4=pastVal33;
               aux5=pastVal34;
                           aux6=pastVal35;
           }
        if(getParentModule()->getIndex()==4){
               aux=pastVal4;
               aux2=pastVal41;
               aux3=pastVal42;
               aux4=pastVal43;
               aux5=pastVal44;
                           aux6=pastVal45;
           }
        if(getParentModule()->getIndex()==5){
               aux=pastVal5;
               aux2=pastVal51;
               aux3=pastVal52;
               aux4=pastVal53;
               aux5=pastVal54;
                           aux6=pastVal55;
           }
        if(getParentModule()->getIndex()==6){
               aux=pastVal6;
               aux2=pastVal61;
               aux3=pastVal62;
               aux4=pastVal63;
               aux5=pastVal64;
                           aux6=pastVal65;
           }
        if(getParentModule()->getIndex()==7){
               aux=pastVal7;
               aux2=pastVal71;
               aux3=pastVal72;
               aux4=pastVal73;
               aux5=pastVal74;
                           aux6=pastVal75;
           }
        if(getParentModule()->getIndex()==8){
               aux=pastVal8;
               aux2=pastVal81;
               aux3=pastVal82;
               aux4=pastVal83;
               aux5=pastVal84;
                           aux6=pastVal85;
           }

        simtime_t a=simTime();
        EV_INFO << "Received packet1: " << payload<< " ruta "<<payload->getRoadId() << " vs "<< aux <<" vs " <<aux2<<" vs "<<aux3<<" tiempo "<<a<<" lastC "<<lastC<<endl;
        //EV_INFO << "Received packet2: " << payload<< " ruta "<<payload->getLength() << " vs "<< aux <<endl;
        //EV_DETAIL << "Content: " << payload->getRoadId().c_str() << endl;

        getParentModule()->getDisplayString().setTagArg("i", 1, "green");


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
            if(getParentModule()->getIndex()==0){
                renv0=renv0+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv0;
                if(!haveForwarded){
                    pastVal05=pastVal04;
                    pastVal04=pastVal03;
                    pastVal03=pastVal02;
                    pastVal02=pastVal01;
                    pastVal01=pastVal0;
                    pastVal0=payload->getRoadId();
                }
            }
            if(getParentModule()->getIndex()==1){
                renv1=renv1+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv1;
                if(!haveForwarded){
                    pastVal15=pastVal14;
                                        pastVal14=pastVal13;
                                pastVal13=pastVal12;
                                pastVal12=pastVal11;
                                pastVal11=pastVal1;
                                pastVal1=payload->getRoadId();
                            }
            }
            if(getParentModule()->getIndex()==2){
                renv2=renv2+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv2;
                if(!haveForwarded){
                    pastVal25=pastVal24;
                                        pastVal24=pastVal23;
                    pastVal23=pastVal22;

                                pastVal22=pastVal21;
                                pastVal21=pastVal2;
                                pastVal2=payload->getRoadId();
                            }
               }
            if(getParentModule()->getIndex()==3){
                renv3=renv3+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv3;
                if(!haveForwarded){
                    pastVal35=pastVal34;
                                        pastVal34=pastVal33;
                    pastVal33=pastVal32;

                                pastVal32=pastVal31;
                                pastVal31=pastVal3;
                                pastVal3=payload->getRoadId();
                            }
               }
            if(getParentModule()->getIndex()==4){
                renv4=renv4+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv4;
                if(!haveForwarded){
                    pastVal45=pastVal44;
                                        pastVal44=pastVal43;
                    pastVal43=pastVal42;

                                pastVal42=pastVal41;
                                pastVal41=pastVal4;
                                pastVal4=payload->getRoadId();
                            }
               }
            if(getParentModule()->getIndex()==5){
                renv5=renv5+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv5;
                if(!haveForwarded){
                    pastVal55=pastVal54;
                                        pastVal54=pastVal53;
                    pastVal53=pastVal52;

                    pastVal52=pastVal51;
                                pastVal51=pastVal5;
                                pastVal5=payload->getRoadId();
                            }
               }
            if(getParentModule()->getIndex()==6){
                renv6=renv6+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv6;
                if(!haveForwarded){
                    pastVal65=pastVal64;
                                        pastVal64=pastVal63;
                    pastVal63=pastVal62;

                                pastVal62=pastVal61;
                                pastVal61=pastVal6;
                                pastVal6=payload->getRoadId();
                            }
               }
            if(getParentModule()->getIndex()==7){
                renv7=renv7+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv7;
                if(!haveForwarded){
                    pastVal75=pastVal74;
                                        pastVal74=pastVal73;
                    pastVal73=pastVal72;

                    pastVal72=pastVal71;
                                pastVal71=pastVal7;
                                pastVal7=payload->getRoadId();
                            }
               }
            if(getParentModule()->getIndex()==8){
                renv8=renv8+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv8;
                if(!haveForwarded){
                    pastVal85=pastVal84;
                                        pastVal84=pastVal83;
                    pastVal83=pastVal82;

                    pastVal82=pastVal81;
                                pastVal81=pastVal8;
                                pastVal8=payload->getRoadId();
                            }
               }
        }
/*

//sim3
    //Se hace equivalencia de las rutas con el carril
        auto payload = pk->peekAtFront<VeinsInetSampleMessage>();

        EV_INFO << "Received packet2: " << payload << " ruta actual " << traciVehicle->getRoadId()<<endl;
        //EV_DETAIL << "Content: " << payload->getRoadId().c_str() << endl;

        getParentModule()->getDisplayString().setTagArg("i", 1, "green");
        const char* aux = "E29";
        //if(getParentModule()->getIndex()==4){
        //Los vehiculos en la seccion de la ruta 29 desaceleran
        if(strcmp(traciVehicle->getRoadId().c_str(), aux)==0){
            traciVehicle->setSpeed(2);
        }

        if (haveForwarded) return;

        auto packet = createPacket("relay");
        packet->insertAtBack(payload);
        sendPacket(std::move(packet));

        haveForwarded = true;
        //getParentModule()->getDisplayString().setTagArg("i", 1, "gray");
*/
}
