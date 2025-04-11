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
double lastC=48; //41 40.5 40.2 sim3 32 sim2 21
//E2 empieza en 47

int env0=0;
int env1=0;
int env2=0;
int env3=0;
int env4=0;
int env5=0;
int env6=0;
int env7=0;
int env8=0;
int env9=0;
int env10=0;
int env11=0;
int env12=0;
int env13=0;
int env14=0;
int env15=0;
int env16=0;
int env17=0;
int env18=0;
int env19=0;
int env20=0;
int env21=0;
int env22=0;
int env23=0;
int env24=0;
int env25=0;
int env26=0;
int env27=0;
int env28=0;
int env29=0;
int env30=0;

int renv0=0;
int renv1=0;
int renv2=0;
int renv3=0;
int renv4=0;
int renv5=0;
int renv6=0;
int renv7=0;
int renv8=0;
int renv9=0;
int renv10=0;
int renv11=0;
int renv12=0;
int renv13=0;
int renv14=0;
int renv15=0;
int renv16=0;
int renv17=0;
int renv18=0;
int renv19=0;
int renv20=0;
int renv21=0;
int renv22=0;
int renv23=0;
int renv24=0;
int renv25=0;
int renv26=0;
int renv27=0;
int renv28=0;
int renv29=0;
int renv30=0;

const char* pastVal00="";
const char* pastVal10="";
const char* pastVal20="";
const char* pastVal30="";
const char* pastVal40="";
const char* pastVal50="";
const char* pastVal60="";
const char* pastVal70="";
const char* pastVal80="";
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

const char* pastVal90="";
const char* pastVal100="";
const char* pastVal110="";
const char* pastVal120="";
const char* pastVal130="";
const char* pastVal140="";
const char* pastVal150="";
const char* pastVal160="";
const char* pastVal170="";
const char* pastVal91="";
const char* pastVal101="";
const char* pastVal111="";
const char* pastVal121="";
const char* pastVal131="";
const char* pastVal141="";
const char* pastVal151="";
const char* pastVal161="";
const char* pastVal171="";
const char* pastVal92="";
const char* pastVal102="";
const char* pastVal112="";
const char* pastVal122="";
const char* pastVal132="";
const char* pastVal142="";
const char* pastVal152="";
const char* pastVal162="";
const char* pastVal172="";
const char* pastVal93="";
const char* pastVal103="";
const char* pastVal113="";
const char* pastVal123="";
const char* pastVal133="";
const char* pastVal143="";
const char* pastVal153="";
const char* pastVal163="";
const char* pastVal173="";
const char* pastVal94="";
const char* pastVal104="";
const char* pastVal114="";
const char* pastVal124="";
const char* pastVal134="";
const char* pastVal144="";
const char* pastVal154="";
const char* pastVal164="";
const char* pastVal174="";
const char* pastVal95="";
const char* pastVal105="";
const char* pastVal115="";
const char* pastVal125="";
const char* pastVal135="";
const char* pastVal145="";
const char* pastVal155="";
const char* pastVal165="";
const char* pastVal175="";

const char* pastVal180="";
const char* pastVal190="";
const char* pastVal200="";
const char* pastVal210="";
const char* pastVal220="";
const char* pastVal230="";
const char* pastVal240="";
const char* pastVal250="";
const char* pastVal260="";
const char* pastVal181="";
const char* pastVal191="";
const char* pastVal201="";
const char* pastVal211="";
const char* pastVal221="";
const char* pastVal231="";
const char* pastVal241="";
const char* pastVal251="";
const char* pastVal261="";
const char* pastVal182="";
const char* pastVal192="";
const char* pastVal202="";
const char* pastVal212="";
const char* pastVal222="";
const char* pastVal232="";
const char* pastVal242="";
const char* pastVal252="";
const char* pastVal262="";
const char* pastVal183="";
const char* pastVal193="";
const char* pastVal203="";
const char* pastVal213="";
const char* pastVal223="";
const char* pastVal233="";
const char* pastVal243="";
const char* pastVal253="";
const char* pastVal263="";
const char* pastVal184="";
const char* pastVal194="";
const char* pastVal204="";
const char* pastVal214="";
const char* pastVal224="";
const char* pastVal234="";
const char* pastVal244="";
const char* pastVal254="";
const char* pastVal264="";
const char* pastVal185="";
const char* pastVal195="";
const char* pastVal205="";
const char* pastVal215="";
const char* pastVal225="";
const char* pastVal235="";
const char* pastVal245="";
const char* pastVal255="";
const char* pastVal265="";

const char* pastVal270="";
const char* pastVal280="";
const char* pastVal290="";
const char* pastVal300="";
const char* pastVal271="";
const char* pastVal281="";
const char* pastVal291="";
const char* pastVal301="";
const char* pastVal272="";
const char* pastVal282="";
const char* pastVal292="";
const char* pastVal302="";
const char* pastVal273="";
const char* pastVal283="";
const char* pastVal293="";
const char* pastVal303="";
const char* pastVal274="";
const char* pastVal284="";
const char* pastVal294="";
const char* pastVal304="";
const char* pastVal275="";
const char* pastVal285="";
const char* pastVal295="";
const char* pastVal305="";

std::vector<const char *> nodo1;
std::vector<const char *> nodo2;
std::vector<const char *> nodo3;
std::vector<const char *> nodo4;
std::vector<const char *> nodo5;
std::vector<const char *> nodo6;
std::vector<const char *> nodo7;
std::vector<const char *> nodo8;
std::vector<const char *> nodo9;
std::vector<const char *> nodo10;
std::vector<const char *> nodo11;
std::vector<const char *> nodo12;
std::vector<const char *> nodo13;
std::vector<const char *> nodo14;
std::vector<const char *> nodo15;
std::vector<const char *> nodo16;
std::vector<const char *> nodo17;
std::vector<const char *> nodo18;
std::vector<const char *> nodo19;
std::vector<const char *> nodo20;
std::vector<const char *> nodo21;
std::vector<const char *> nodo22;
std::vector<const char *> nodo23;
std::vector<const char *> nodo24;
std::vector<const char *> nodo25;
std::vector<const char *> nodo26;
std::vector<const char *> nodo27;
std::vector<const char *> nodo28;
std::vector<const char *> nodo29;
std::vector<const char *> nodo30;
std::vector<const char *> nodo0;

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
                                            if(getParentModule()->getIndex()==9){
                                                env9=env9+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env9;

                                               }
                                            if(getParentModule()->getIndex()==10){
                                                env10=env10+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env10;

                                               }
                                            if(getParentModule()->getIndex()==11){
                                                env11=env11+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env11;

                                               }
                                            if(getParentModule()->getIndex()==12){
                                                env12=env12+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env12;

                                               }
                                            if(getParentModule()->getIndex()==13){
                                                env13=env13+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env13;

                                               }
                                            if(getParentModule()->getIndex()==14){
                                                env14=env14+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env14;

                                               }
                                            if(getParentModule()->getIndex()==15){
                                                env15=env15+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env15;

                                               }
                                            if(getParentModule()->getIndex()==16){
                                                env16=env16+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env16;

                                               }
                                            if(getParentModule()->getIndex()==17){
                                                env17=env17+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env17;

                                               }
                                            if(getParentModule()->getIndex()==18){
                                                env18=env18+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env18;

                                               }
                                            if(getParentModule()->getIndex()==19){
                                                env19=env19+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env19;

                                               }
                                            if(getParentModule()->getIndex()==20){
                                                env20=env20+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env20;

                                               }
                                            if(getParentModule()->getIndex()==21){
                                                env21=env21+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env21;

                                               }
                                            if(getParentModule()->getIndex()==22){
                                                env22=env22+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env22;

                                               }
                                            if(getParentModule()->getIndex()==23){
                                                env23=env23+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env23;

                                               }
                                            if(getParentModule()->getIndex()==24){
                                                env24=env24+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env24;

                                               }
                                            if(getParentModule()->getIndex()==25){
                                                env25=env25+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env25;

                                               }
                                            if(getParentModule()->getIndex()==26){
                                                env26=env26+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env26;

                                               }
                                            if(getParentModule()->getIndex()==27){
                                                env27=env27+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env27;

                                               }
                                            if(getParentModule()->getIndex()==28){
                                                env28=env28+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env28;

                                               }
                                            if(getParentModule()->getIndex()==29){
                                                env29=env29+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env29;

                                               }
                                            if(getParentModule()->getIndex()==30){
                                                env30=env30+1;
                                                EV_INFO << "Envia1 " << getParentModule()->getIndex()<< " num "<<env30;

                                               }

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

/*
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
            lastC=lastC+1;//1 0.5 0.2

            pastVal00="";
            pastVal10="";
            pastVal20="";
            pastVal30="";
            pastVal40="";
            pastVal50="";
            pastVal60="";
            pastVal70="";
            pastVal80="";
            pastVal90="";
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

                   pastVal90="";
                   pastVal100="";
                   pastVal110="";
                   pastVal120="";
                   pastVal130="";
                   pastVal140="";
                   pastVal150="";
                   pastVal160="";
                   pastVal170="";
                   pastVal91="";
                   pastVal101="";
                   pastVal111="";
                   pastVal121="";
                   pastVal131="";
                   pastVal141="";
                   pastVal151="";
                   pastVal161="";
                   pastVal171="";
                   pastVal92="";
                   pastVal102="";
                   pastVal112="";
                   pastVal122="";
                   pastVal132="";
                   pastVal142="";
                   pastVal152="";
                   pastVal162="";
                   pastVal172="";
                   pastVal93="";
                  pastVal103="";
                  pastVal113="";
                  pastVal123="";
                  pastVal133="";
                  pastVal143="";
                  pastVal153="";
                  pastVal163="";
                  pastVal173="";
                  pastVal94="";
                  pastVal104="";
                  pastVal114="";
                  pastVal124="";
                  pastVal134="";
                  pastVal144="";
                  pastVal154="";
                  pastVal164="";
                  pastVal174="";
                  pastVal95="";
                  pastVal105="";
                  pastVal115="";
                  pastVal125="";
                  pastVal135="";
                  pastVal145="";
                  pastVal155="";
                  pastVal165="";
                  pastVal175="";

                  pastVal180="";
                 pastVal190="";
                 pastVal200="";
                 pastVal210="";
                 pastVal220="";
                 pastVal230="";
                 pastVal240="";
                 pastVal250="";
                 pastVal260="";
                 pastVal181="";
                 pastVal191="";
                 pastVal201="";
                 pastVal211="";
                 pastVal221="";
                 pastVal231="";
                 pastVal241="";
                 pastVal251="";
                 pastVal261="";
                 pastVal182="";
                 pastVal192="";
                 pastVal202="";
                 pastVal212="";
                 pastVal222="";
                 pastVal232="";
                 pastVal242="";
                 pastVal252="";
                 pastVal262="";
                 pastVal183="";
                pastVal193="";
                pastVal203="";
                pastVal213="";
                pastVal223="";
                pastVal233="";
                pastVal243="";
                pastVal253="";
                pastVal263="";
                pastVal184="";
                pastVal194="";
                pastVal204="";
                pastVal214="";
                pastVal224="";
                pastVal234="";
                pastVal244="";
                pastVal254="";
                pastVal264="";
                pastVal185="";
                pastVal195="";
                pastVal205="";
                pastVal215="";
                pastVal225="";
                pastVal235="";
                pastVal245="";
                pastVal255="";
                pastVal265="";

                pastVal270="";
               pastVal280="";
               pastVal290="";
               pastVal300="";
               pastVal271="";
               pastVal281="";
               pastVal291="";
               pastVal301="";
               pastVal272="";
               pastVal282="";
               pastVal292="";
               pastVal302="";
               pastVal273="";
              pastVal283="";
              pastVal293="";
              pastVal303="";
              pastVal274="";
              pastVal284="";
              pastVal294="";
              pastVal304="";
              pastVal275="";
              pastVal285="";
              pastVal295="";
              pastVal305="";

        }

        if(getParentModule()->getIndex()==0){
            aux=pastVal00;
            aux2=pastVal01;
            aux3=pastVal02;
            aux4=pastVal03;
            aux5=pastVal04;
            aux6=pastVal05;

        }
        if(getParentModule()->getIndex()==1){
               aux=pastVal10;
               aux2=pastVal11;
               aux3=pastVal12;
               aux4=pastVal13;
               aux5=pastVal14;
                           aux6=pastVal15;
           }
        if(getParentModule()->getIndex()==2){
               aux=pastVal20;
               aux2=pastVal21;
               aux3=pastVal22;
               aux4=pastVal23;
               aux5=pastVal24;
                           aux6=pastVal25;
           }
        if(getParentModule()->getIndex()==3){
               aux=pastVal30;
               aux2=pastVal31;
               aux3=pastVal32;
               aux4=pastVal33;
               aux5=pastVal34;
                           aux6=pastVal35;
           }
        if(getParentModule()->getIndex()==4){
               aux=pastVal40;
               aux2=pastVal41;
               aux3=pastVal42;
               aux4=pastVal43;
               aux5=pastVal44;
                           aux6=pastVal45;
           }
        if(getParentModule()->getIndex()==5){
               aux=pastVal50;
               aux2=pastVal51;
               aux3=pastVal52;
               aux4=pastVal53;
               aux5=pastVal54;
                           aux6=pastVal55;
           }
        if(getParentModule()->getIndex()==6){
               aux=pastVal60;
               aux2=pastVal61;
               aux3=pastVal62;
               aux4=pastVal63;
               aux5=pastVal64;
                           aux6=pastVal65;
           }
        if(getParentModule()->getIndex()==7){
               aux=pastVal70;
               aux2=pastVal71;
               aux3=pastVal72;
               aux4=pastVal73;
               aux5=pastVal74;
                           aux6=pastVal75;
           }
        if(getParentModule()->getIndex()==8){
               aux=pastVal80;
               aux2=pastVal81;
               aux3=pastVal82;
               aux4=pastVal83;
               aux5=pastVal84;
                           aux6=pastVal85;
           }
        if(getParentModule()->getIndex()==9){
                       aux=pastVal90;
                       aux2=pastVal91;
                       aux3=pastVal92;
                       aux4=pastVal93;
                       aux5=pastVal94;
                                   aux6=pastVal95;
         }
        if(getParentModule()->getIndex()==10){
                       aux=pastVal100;
                       aux2=pastVal101;
                       aux3=pastVal102;
                       aux4=pastVal103;
                       aux5=pastVal104;
                       aux6=pastVal105;
                   }
        if(getParentModule()->getIndex()==11){
                               aux=pastVal110;
                               aux2=pastVal111;
                               aux3=pastVal112;
                               aux4=pastVal113;
                               aux5=pastVal114;
                               aux6=pastVal115;
                           }
        if(getParentModule()->getIndex()==12){
                               aux=pastVal120;
                               aux2=pastVal121;
                               aux3=pastVal122;
                               aux4=pastVal123;
                               aux5=pastVal124;
                               aux6=pastVal125;
                           }
        if(getParentModule()->getIndex()==13){
                               aux=pastVal130;
                               aux2=pastVal131;
                               aux3=pastVal132;
                               aux4=pastVal133;
                               aux5=pastVal134;
                               aux6=pastVal135;
                           }
        if(getParentModule()->getIndex()==14){
                               aux=pastVal140;
                               aux2=pastVal141;
                               aux3=pastVal142;
                               aux4=pastVal143;
                               aux5=pastVal144;
                               aux6=pastVal145;
                           }
        if(getParentModule()->getIndex()==15){
                               aux=pastVal150;
                               aux2=pastVal151;
                               aux3=pastVal152;
                               aux4=pastVal153;
                               aux5=pastVal154;
                               aux6=pastVal155;
                           }
        if(getParentModule()->getIndex()==16){
                               aux=pastVal160;
                               aux2=pastVal161;
                               aux3=pastVal162;
                               aux4=pastVal163;
                               aux5=pastVal164;
                               aux6=pastVal165;
                           }
        if(getParentModule()->getIndex()==17){
                               aux=pastVal170;
                               aux2=pastVal171;
                               aux3=pastVal172;
                               aux4=pastVal173;
                               aux5=pastVal174;
                               aux6=pastVal175;
                           }
        if(getParentModule()->getIndex()==18){
                               aux=pastVal180;
                               aux2=pastVal181;
                               aux3=pastVal182;
                               aux4=pastVal183;
                               aux5=pastVal184;
                               aux6=pastVal185;
                           }
        if(getParentModule()->getIndex()==19){
                               aux=pastVal190;
                               aux2=pastVal191;
                               aux3=pastVal192;
                               aux4=pastVal193;
                               aux5=pastVal194;
                               aux6=pastVal195;
                           }
        if(getParentModule()->getIndex()==20){
                               aux=pastVal200;
                               aux2=pastVal201;
                               aux3=pastVal202;
                               aux4=pastVal203;
                               aux5=pastVal204;
                               aux6=pastVal205;
                           }
        if(getParentModule()->getIndex()==21){
                               aux=pastVal210;
                               aux2=pastVal211;
                               aux3=pastVal212;
                               aux4=pastVal213;
                               aux5=pastVal214;
                               aux6=pastVal215;
                           }
        if(getParentModule()->getIndex()==22){
                               aux=pastVal220;
                               aux2=pastVal221;
                               aux3=pastVal222;
                               aux4=pastVal223;
                               aux5=pastVal224;
                               aux6=pastVal225;
                           }
        if(getParentModule()->getIndex()==23){
                               aux=pastVal230;
                               aux2=pastVal231;
                               aux3=pastVal232;
                               aux4=pastVal233;
                               aux5=pastVal234;
                               aux6=pastVal235;
                           }
        if(getParentModule()->getIndex()==24){
                               aux=pastVal240;
                               aux2=pastVal241;
                               aux3=pastVal242;
                               aux4=pastVal243;
                               aux5=pastVal244;
                               aux6=pastVal245;
                           }
        if(getParentModule()->getIndex()==25){
                               aux=pastVal250;
                               aux2=pastVal251;
                               aux3=pastVal252;
                               aux4=pastVal253;
                               aux5=pastVal254;
                               aux6=pastVal255;
                           }
        if(getParentModule()->getIndex()==26){
                               aux=pastVal260;
                               aux2=pastVal261;
                               aux3=pastVal262;
                               aux4=pastVal263;
                               aux5=pastVal264;
                               aux6=pastVal265;
                           }
        if(getParentModule()->getIndex()==27){
                               aux=pastVal270;
                               aux2=pastVal271;
                               aux3=pastVal272;
                               aux4=pastVal273;
                               aux5=pastVal274;
                              aux6=pastVal275;
                           }
        if(getParentModule()->getIndex()==28){
                               aux=pastVal280;
                               aux2=pastVal281;
                               aux3=pastVal282;
                               aux4=pastVal283;
                               aux5=pastVal284;
                               aux6=pastVal285;
                           }
        if(getParentModule()->getIndex()==29){
                               aux=pastVal290;
                               aux2=pastVal291;
                               aux3=pastVal292;
                               aux4=pastVal293;
                               aux5=pastVal294;
                               aux6=pastVal295;
                           }
        if(getParentModule()->getIndex()==30){
                               aux=pastVal300;
                               aux2=pastVal301;
                               aux3=pastVal302;
                               aux4=pastVal303;
                               aux5=pastVal304;
                               aux6=pastVal305;
                           }



        simtime_t a=simTime();
        EV_INFO << "Received packet3: " << payload<< " ruta "<<payload->getRoadId() << " vs "<< aux <<" vs " <<aux2<<" vs "<<aux3<<" tiempo "<<a<<" lastC "<<lastC<<endl;
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
                    pastVal01=pastVal00;
                    pastVal00=payload->getRoadId();
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
                                pastVal11=pastVal10;
                               pastVal10=payload->getRoadId();
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
                                pastVal21=pastVal20;
                                pastVal20=payload->getRoadId();
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
                                pastVal31=pastVal30;
                                pastVal30=payload->getRoadId();
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
                                pastVal41=pastVal40;
                                pastVal40=payload->getRoadId();
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
                                pastVal51=pastVal50;
                                pastVal50=payload->getRoadId();
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
                                pastVal61=pastVal60;
                                pastVal60=payload->getRoadId();
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
                                pastVal71=pastVal70;
                                pastVal70=payload->getRoadId();
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
                                pastVal81=pastVal80;
                                pastVal80=payload->getRoadId();
                            }
               }


        if(getParentModule()->getIndex()==9){
                       renv9=renv9+1;
                       EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv9;
                       if(!haveForwarded){
                           pastVal95=pastVal94;
                           pastVal94=pastVal93;
                           pastVal93=pastVal92;
                           pastVal92=pastVal91;
                           pastVal91=pastVal90;
                           pastVal90=payload->getRoadId();
                                   }
                      }
        if(getParentModule()->getIndex()==10){
                               renv10=renv10+1;
                               EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv10;
                               if(!haveForwarded){
                                   pastVal105=pastVal104;
                                   pastVal104=pastVal103;
                                   pastVal103=pastVal102;
                                   pastVal102=pastVal101;
                                   pastVal101=pastVal100;
                                   pastVal100=payload->getRoadId();
                                           }
                              }
        if(getParentModule()->getIndex()==11){
                                    renv11=renv11+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv11;
                                    if(!haveForwarded){
                                        pastVal115=pastVal114;
                                        pastVal114=pastVal113;
                                        pastVal113=pastVal112;
                                        pastVal112=pastVal111;
                                        pastVal111=pastVal110;
                                        pastVal110=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==12){
                                    renv12=renv12+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv12;
                                    if(!haveForwarded){
                                        pastVal125=pastVal124;
                                        pastVal124=pastVal123;
                                        pastVal123=pastVal122;
                                        pastVal122=pastVal121;
                                        pastVal121=pastVal120;
                                        pastVal120=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==13){
                                    renv13=renv13+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv13;
                                    if(!haveForwarded){
                                        pastVal135=pastVal134;
                                        pastVal134=pastVal133;
                                        pastVal133=pastVal132;
                                        pastVal132=pastVal131;
                                        pastVal131=pastVal130;
                                        pastVal130=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==14){
                                    renv14=renv14+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv14;
                                    if(!haveForwarded){
                                        pastVal145=pastVal144;
                                        pastVal144=pastVal143;
                                        pastVal143=pastVal142;
                                        pastVal142=pastVal141;
                                        pastVal141=pastVal140;
                                        pastVal140=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==15){
                                    renv15=renv15+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv15;
                                    if(!haveForwarded){
                                        pastVal155=pastVal154;
                                        pastVal154=pastVal153;
                                        pastVal153=pastVal152;
                                        pastVal152=pastVal151;
                                        pastVal151=pastVal150;
                                        pastVal150=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==16){
                                    renv16=renv16+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv16;
                                    if(!haveForwarded){
                                        pastVal165=pastVal164;
                                        pastVal164=pastVal163;
                                        pastVal163=pastVal162;
                                        pastVal162=pastVal161;
                                        pastVal161=pastVal160;
                                        pastVal160=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==17){
                                    renv17=renv17+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv17;
                                    if(!haveForwarded){
                                        pastVal175=pastVal174;
                                        pastVal174=pastVal173;
                                        pastVal173=pastVal172;
                                        pastVal172=pastVal171;
                                        pastVal171=pastVal170;
                                        pastVal170=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==18){
                                    renv18=renv18+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv18;
                                    if(!haveForwarded){
                                        pastVal185=pastVal184;
                                        pastVal184=pastVal183;
                                        pastVal183=pastVal182;
                                        pastVal182=pastVal181;
                                        pastVal181=pastVal180;
                                        pastVal180=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==19){
                                    renv19=renv19+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv19;
                                    if(!haveForwarded){
                                        pastVal195=pastVal194;
                                        pastVal194=pastVal193;
                                        pastVal193=pastVal192;
                                        pastVal192=pastVal191;
                                        pastVal191=pastVal190;
                                        pastVal190=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==20){
                                    renv20=renv20+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv20;
                                    if(!haveForwarded){
                                        pastVal205=pastVal204;
                                        pastVal204=pastVal203;
                                        pastVal203=pastVal202;
                                        pastVal202=pastVal201;
                                        pastVal201=pastVal200;
                                        pastVal200=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==21){
                                    renv21=renv21+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv21;
                                    if(!haveForwarded){
                                        pastVal215=pastVal214;
                                        pastVal214=pastVal213;
                                        pastVal213=pastVal212;
                                        pastVal212=pastVal211;
                                        pastVal211=pastVal210;
                                        pastVal210=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==22){
                                    renv22=renv22+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv22;
                                    if(!haveForwarded){
                                        pastVal225=pastVal224;
                                        pastVal224=pastVal223;
                                        pastVal223=pastVal222;
                                        pastVal222=pastVal221;
                                        pastVal221=pastVal220;
                                        pastVal220=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==23){
                                    renv23=renv23+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv23;
                                    if(!haveForwarded){
                                        pastVal235=pastVal234;
                                        pastVal234=pastVal233;
                                        pastVal233=pastVal232;
                                        pastVal232=pastVal231;
                                        pastVal231=pastVal230;
                                        pastVal230=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==24){
                                    renv24=renv24+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv24;
                                    if(!haveForwarded){
                                        pastVal245=pastVal244;
                                        pastVal244=pastVal243;
                                        pastVal243=pastVal242;
                                        pastVal242=pastVal241;
                                        pastVal241=pastVal240;
                                        pastVal240=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==25){
                                    renv25=renv25+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv25;
                                    if(!haveForwarded){
                                        pastVal255=pastVal254;
                                        pastVal254=pastVal253;
                                        pastVal253=pastVal252;
                                        pastVal252=pastVal251;
                                        pastVal251=pastVal250;
                                        pastVal250=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==26){
                                    renv26=renv26+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv26;
                                    if(!haveForwarded){
                                        pastVal265=pastVal264;
                                        pastVal264=pastVal263;
                                        pastVal263=pastVal262;
                                        pastVal262=pastVal261;
                                        pastVal261=pastVal260;
                                        pastVal260=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==27){
                                    renv27=renv27+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv27;
                                    if(!haveForwarded){
                                        pastVal275=pastVal274;
                                        pastVal274=pastVal273;
                                        pastVal273=pastVal272;
                                        pastVal272=pastVal271;
                                        pastVal271=pastVal270;
                                        pastVal270=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==28){
                                    renv28=renv28+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv28;
                                    if(!haveForwarded){
                                        pastVal285=pastVal284;
                                        pastVal284=pastVal283;
                                        pastVal283=pastVal282;
                                        pastVal282=pastVal281;
                                        pastVal281=pastVal280;
                                        pastVal280=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==29){
                                    renv29=renv29+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv29;
                                    if(!haveForwarded){
                                        pastVal295=pastVal294;
                                        pastVal294=pastVal293;
                                        pastVal293=pastVal292;
                                        pastVal292=pastVal291;
                                        pastVal291=pastVal290;
                                        pastVal290=payload->getRoadId();
                                                }
                                   }
        if(getParentModule()->getIndex()==30){
                                    renv30=renv30+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv30;
                                    if(!haveForwarded){
                                        pastVal305=pastVal304;
                                        pastVal304=pastVal303;
                                        pastVal303=pastVal302;
                                        pastVal302=pastVal301;
                                        pastVal301=pastVal300;
                                        pastVal300=payload->getRoadId();
                                                }
                                   }
            }
*/
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

    //sim4

        int contR=1;
        //getParentModule()->getDisplayString().setTagArg("i", 1, "gray");

        auto payload = pk->peekAtFront<VeinsInetSampleMessage>();
        double t2 = simTime().dbl();
        if(lastC+0.0<t2){
            lastC=lastC+1;//1 0.5 0.2

            nodo0.clear();
            nodo1.clear();
            nodo2.clear();
            nodo3.clear();
            nodo4.clear();
            nodo5.clear();
            nodo6.clear();
            nodo7.clear();
            nodo8.clear();
            nodo9.clear();
            nodo10.clear();
            nodo11.clear();
            nodo12.clear();
            nodo13.clear();
            nodo14.clear();
            nodo15.clear();
            nodo16.clear();
            nodo17.clear();
            nodo18.clear();
            nodo19.clear();
            nodo20.clear();
            nodo21.clear();
            nodo22.clear();
            nodo23.clear();
            nodo24.clear();
            nodo25.clear();
            nodo26.clear();
            nodo27.clear();
            nodo28.clear();
            nodo29.clear();
            nodo30.clear();


        }
/*
        if(getParentModule()->getIndex()==0){
            aux=pastVal00;
            aux2=pastVal01;
            aux3=pastVal02;
            aux4=pastVal03;
            aux5=pastVal04;
            aux6=pastVal05;

        }
        if(getParentModule()->getIndex()==1){
               aux=pastVal10;
               aux2=pastVal11;
               aux3=pastVal12;
               aux4=pastVal13;
               aux5=pastVal14;
                           aux6=pastVal15;
           }
        if(getParentModule()->getIndex()==2){
               aux=pastVal20;
               aux2=pastVal21;
               aux3=pastVal22;
               aux4=pastVal23;
               aux5=pastVal24;
                           aux6=pastVal25;
           }
        if(getParentModule()->getIndex()==3){
               aux=pastVal30;
               aux2=pastVal31;
               aux3=pastVal32;
               aux4=pastVal33;
               aux5=pastVal34;
                           aux6=pastVal35;
           }
        if(getParentModule()->getIndex()==4){
               aux=pastVal40;
               aux2=pastVal41;
               aux3=pastVal42;
               aux4=pastVal43;
               aux5=pastVal44;
                           aux6=pastVal45;
           }
        if(getParentModule()->getIndex()==5){
               aux=pastVal50;
               aux2=pastVal51;
               aux3=pastVal52;
               aux4=pastVal53;
               aux5=pastVal54;
                           aux6=pastVal55;
           }
        if(getParentModule()->getIndex()==6){
               aux=pastVal60;
               aux2=pastVal61;
               aux3=pastVal62;
               aux4=pastVal63;
               aux5=pastVal64;
                           aux6=pastVal65;
           }
        if(getParentModule()->getIndex()==7){
               aux=pastVal70;
               aux2=pastVal71;
               aux3=pastVal72;
               aux4=pastVal73;
               aux5=pastVal74;
                           aux6=pastVal75;
           }
        if(getParentModule()->getIndex()==8){
               aux=pastVal80;
               aux2=pastVal81;
               aux3=pastVal82;
               aux4=pastVal83;
               aux5=pastVal84;
                           aux6=pastVal85;
           }
        if(getParentModule()->getIndex()==9){
                       aux=pastVal90;
                       aux2=pastVal91;
                       aux3=pastVal92;
                       aux4=pastVal93;
                       aux5=pastVal94;
                                   aux6=pastVal95;
         }
        if(getParentModule()->getIndex()==10){
                       aux=pastVal100;
                       aux2=pastVal101;
                       aux3=pastVal102;
                       aux4=pastVal103;
                       aux5=pastVal104;
                       aux6=pastVal105;
                   }
        if(getParentModule()->getIndex()==11){
                               aux=pastVal110;
                               aux2=pastVal111;
                               aux3=pastVal112;
                               aux4=pastVal113;
                               aux5=pastVal114;
                               aux6=pastVal115;
                           }
        if(getParentModule()->getIndex()==12){
                               aux=pastVal120;
                               aux2=pastVal121;
                               aux3=pastVal122;
                               aux4=pastVal123;
                               aux5=pastVal124;
                               aux6=pastVal125;
                           }
        if(getParentModule()->getIndex()==13){
                               aux=pastVal130;
                               aux2=pastVal131;
                               aux3=pastVal132;
                               aux4=pastVal133;
                               aux5=pastVal134;
                               aux6=pastVal135;
                           }
        if(getParentModule()->getIndex()==14){
                               aux=pastVal140;
                               aux2=pastVal141;
                               aux3=pastVal142;
                               aux4=pastVal143;
                               aux5=pastVal144;
                               aux6=pastVal145;
                           }
        if(getParentModule()->getIndex()==15){
                               aux=pastVal150;
                               aux2=pastVal151;
                               aux3=pastVal152;
                               aux4=pastVal153;
                               aux5=pastVal154;
                               aux6=pastVal155;
                           }
        if(getParentModule()->getIndex()==16){
                               aux=pastVal160;
                               aux2=pastVal161;
                               aux3=pastVal162;
                               aux4=pastVal163;
                               aux5=pastVal164;
                               aux6=pastVal165;
                           }
        if(getParentModule()->getIndex()==17){
                               aux=pastVal170;
                               aux2=pastVal171;
                               aux3=pastVal172;
                               aux4=pastVal173;
                               aux5=pastVal174;
                               aux6=pastVal175;
                           }
        if(getParentModule()->getIndex()==18){
                               aux=pastVal180;
                               aux2=pastVal181;
                               aux3=pastVal182;
                               aux4=pastVal183;
                               aux5=pastVal184;
                               aux6=pastVal185;
                           }
        if(getParentModule()->getIndex()==19){
                               aux=pastVal190;
                               aux2=pastVal191;
                               aux3=pastVal192;
                               aux4=pastVal193;
                               aux5=pastVal194;
                               aux6=pastVal195;
                           }
        if(getParentModule()->getIndex()==20){
                               aux=pastVal200;
                               aux2=pastVal201;
                               aux3=pastVal202;
                               aux4=pastVal203;
                               aux5=pastVal204;
                               aux6=pastVal205;
                           }
        if(getParentModule()->getIndex()==21){
                               aux=pastVal210;
                               aux2=pastVal211;
                               aux3=pastVal212;
                               aux4=pastVal213;
                               aux5=pastVal214;
                               aux6=pastVal215;
                           }
        if(getParentModule()->getIndex()==22){
                               aux=pastVal220;
                               aux2=pastVal221;
                               aux3=pastVal222;
                               aux4=pastVal223;
                               aux5=pastVal224;
                               aux6=pastVal225;
                           }
        if(getParentModule()->getIndex()==23){
                               aux=pastVal230;
                               aux2=pastVal231;
                               aux3=pastVal232;
                               aux4=pastVal233;
                               aux5=pastVal234;
                               aux6=pastVal235;
                           }
        if(getParentModule()->getIndex()==24){
                               aux=pastVal240;
                               aux2=pastVal241;
                               aux3=pastVal242;
                               aux4=pastVal243;
                               aux5=pastVal244;
                               aux6=pastVal245;
                           }
        if(getParentModule()->getIndex()==25){
                               aux=pastVal250;
                               aux2=pastVal251;
                               aux3=pastVal252;
                               aux4=pastVal253;
                               aux5=pastVal254;
                               aux6=pastVal255;
                           }
        if(getParentModule()->getIndex()==26){
                               aux=pastVal260;
                               aux2=pastVal261;
                               aux3=pastVal262;
                               aux4=pastVal263;
                               aux5=pastVal264;
                               aux6=pastVal265;
                           }
        if(getParentModule()->getIndex()==27){
                               aux=pastVal270;
                               aux2=pastVal271;
                               aux3=pastVal272;
                               aux4=pastVal273;
                               aux5=pastVal274;
                              aux6=pastVal275;
                           }
        if(getParentModule()->getIndex()==28){
                               aux=pastVal280;
                               aux2=pastVal281;
                               aux3=pastVal282;
                               aux4=pastVal283;
                               aux5=pastVal284;
                               aux6=pastVal285;
                           }
        if(getParentModule()->getIndex()==29){
                               aux=pastVal290;
                               aux2=pastVal291;
                               aux3=pastVal292;
                               aux4=pastVal293;
                               aux5=pastVal294;
                               aux6=pastVal295;
                           }
        if(getParentModule()->getIndex()==30){
                               aux=pastVal300;
                               aux2=pastVal301;
                               aux3=pastVal302;
                               aux4=pastVal303;
                               aux5=pastVal304;
                               aux6=pastVal305;
                           }

*/

        simtime_t a=simTime();
        EV_INFO << "Received packet0: " << payload<< " ruta "<<payload->getRoadId() << " tiempo "<<a<<" lastC "<<lastC<<endl;
        //EV_INFO << "Received packet2: " << payload<< " ruta "<<payload->getLength() << " vs "<< aux <<endl;
        //EV_DETAIL << "Content: " << payload->getRoadId().c_str() << endl;

        getParentModule()->getDisplayString().setTagArg("i", 1, "green");

        if (getParentModule()->getIndex()==0){
            if (std::find(nodo0.begin(), nodo0.end(), payload->getRoadId()) != nodo0.end()){
                haveForwarded = true;

            }
            else{
                haveForwarded = false;

            }
        }

        if (getParentModule()->getIndex()==1){
            if (std::find(nodo1.begin(), nodo1.end(), payload->getRoadId()) != nodo1.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==2){
            if (std::find(nodo2.begin(), nodo2.end(), payload->getRoadId()) != nodo2.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==3){
            if (std::find(nodo3.begin(), nodo3.end(), payload->getRoadId()) != nodo3.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==4){
            if (std::find(nodo4.begin(), nodo4.end(), payload->getRoadId()) != nodo4.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==5){
            if (std::find(nodo5.begin(), nodo5.end(), payload->getRoadId()) != nodo5.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }
        if (getParentModule()->getIndex()==6){
            if (std::find(nodo6.begin(), nodo6.end(), payload->getRoadId()) != nodo6.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }
        if (getParentModule()->getIndex()==7){
            if (std::find(nodo7.begin(), nodo7.end(), payload->getRoadId()) != nodo7.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==8){
            if (std::find(nodo8.begin(), nodo8.end(), payload->getRoadId()) != nodo8.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==9){
            if (std::find(nodo9.begin(), nodo9.end(), payload->getRoadId()) != nodo9.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==10){
            if (std::find(nodo10.begin(), nodo10.end(), payload->getRoadId()) != nodo10.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==11){
            if (std::find(nodo11.begin(), nodo11.end(), payload->getRoadId()) != nodo11.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==12){
            if (std::find(nodo12.begin(), nodo12.end(), payload->getRoadId()) != nodo12.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==13){
            if (std::find(nodo13.begin(), nodo13.end(), payload->getRoadId()) != nodo13.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==14){
            if (std::find(nodo14.begin(), nodo14.end(), payload->getRoadId()) != nodo14.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==15){
            if (std::find(nodo15.begin(), nodo15.end(), payload->getRoadId()) != nodo15.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==16){
            if (std::find(nodo16.begin(), nodo16.end(), payload->getRoadId()) != nodo16.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==17){
            if (std::find(nodo17.begin(), nodo17.end(), payload->getRoadId()) != nodo17.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }
        if (getParentModule()->getIndex()==18){
            if (std::find(nodo18.begin(), nodo18.end(), payload->getRoadId()) != nodo18.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==19){
            if (std::find(nodo19.begin(), nodo19.end(), payload->getRoadId()) != nodo19.end()){
                haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==20){
            if (std::find(nodo20.begin(), nodo20.end(), payload->getRoadId()) != nodo20.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==21){
            if (std::find(nodo21.begin(), nodo21.end(), payload->getRoadId()) != nodo21.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==22){
            if (std::find(nodo22.begin(), nodo22.end(), payload->getRoadId()) != nodo22.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==23){
            if (std::find(nodo23.begin(), nodo23.end(), payload->getRoadId()) != nodo23.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==24){
            if (std::find(nodo24.begin(), nodo24.end(), payload->getRoadId()) != nodo24.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }
        if (getParentModule()->getIndex()==25){
            if (std::find(nodo25.begin(), nodo25.end(), payload->getRoadId()) != nodo25.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==26){
            if (std::find(nodo26.begin(), nodo26.end(), payload->getRoadId()) != nodo26.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==27){
            if (std::find(nodo27.begin(), nodo27.end(), payload->getRoadId()) != nodo27.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==28){
            if (std::find(nodo28.begin(), nodo28.end(), payload->getRoadId()) != nodo28.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==29){
            if (std::find(nodo29.begin(), nodo29.end(), payload->getRoadId()) != nodo29.end()){
                    haveForwarded = true;

                }
                else{
                    haveForwarded = false;

                }
        }

        if (getParentModule()->getIndex()==30){
            if (std::find(nodo30.begin(), nodo30.end(), payload->getRoadId()) != nodo30.end()){
                            haveForwarded = true;

                        }
                        else{
                            haveForwarded = false;

                        }
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
            if(getParentModule()->getIndex()==0){
                renv0=renv0+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv0;
                nodo0.push_back(payload->getRoadId());
                nodo0.push_back("NO");

            }
            if(getParentModule()->getIndex()==1){
                renv1=renv1+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv1;
                nodo1.push_back(payload->getRoadId());
                nodo1.push_back("NO");

            }
            if(getParentModule()->getIndex()==2){
                renv2=renv2+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv2;
                nodo2.push_back(payload->getRoadId());
                nodo2.push_back("NO");

               }
            if(getParentModule()->getIndex()==3){
                renv3=renv3+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv3;
                nodo3.push_back(payload->getRoadId());
                nodo3.push_back("NO");

               }
            if(getParentModule()->getIndex()==4){
                renv4=renv4+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv4;
                nodo4.push_back(payload->getRoadId());
                nodo4.push_back("NO");

               }
            if(getParentModule()->getIndex()==5){
                renv5=renv5+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv5;
                nodo5.push_back(payload->getRoadId());
                nodo5.push_back("NO");

               }
            if(getParentModule()->getIndex()==6){
                renv6=renv6+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv6;
                nodo6.push_back(payload->getRoadId());
                nodo6.push_back("NO");

               }
            if(getParentModule()->getIndex()==7){
                renv7=renv7+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv7;
                nodo7.push_back(payload->getRoadId());
                nodo7.push_back("NO");

               }
            if(getParentModule()->getIndex()==8){
                renv8=renv8+1;
                EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv8;
                nodo8.push_back(payload->getRoadId());
                nodo8.push_back("NO");

               }


        if(getParentModule()->getIndex()==9){
                       renv9=renv9+1;
                       EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv9;
                       nodo9.push_back(payload->getRoadId());
                       nodo9.push_back("NO");

                      }
        if(getParentModule()->getIndex()==10){
                               renv10=renv10+1;
                               EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv10;
                               nodo10.push_back(payload->getRoadId());
                               nodo10.push_back("NO");

                              }
        if(getParentModule()->getIndex()==11){
                                    renv11=renv11+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv11;
                                    nodo11.push_back(payload->getRoadId());
                                    nodo11.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==12){
                                    renv12=renv12+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv12;
                                    nodo12.push_back(payload->getRoadId());
                                    nodo12.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==13){
                                    renv13=renv13+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv13;
                                    nodo13.push_back(payload->getRoadId());
                                    nodo13.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==14){
                                    renv14=renv14+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv14;
                                    nodo14.push_back(payload->getRoadId());
                                    nodo14.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==15){
                                    renv15=renv15+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv15;
                                    nodo15.push_back(payload->getRoadId());
                                    nodo15.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==16){
                                    renv16=renv16+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv16;
                                    nodo16.push_back(payload->getRoadId());
                                    nodo16.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==17){
                                    renv17=renv17+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv17;
                                    nodo17.push_back(payload->getRoadId());
                                    nodo17.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==18){
                                    renv18=renv18+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv18;
                                    nodo18.push_back(payload->getRoadId());
                                    nodo18.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==19){
                                    renv19=renv19+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv19;
                                    nodo19.push_back(payload->getRoadId());
                                    nodo19.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==20){
                                    renv20=renv20+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv20;
                                    nodo20.push_back(payload->getRoadId());
                                    nodo20.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==21){
                                    renv21=renv21+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv21;
                                    nodo21.push_back(payload->getRoadId());
                                    nodo21.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==22){
                                    renv22=renv22+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv22;
                                    nodo22.push_back(payload->getRoadId());
                                    nodo22.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==23){
                                    renv23=renv23+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv23;
                                    nodo23.push_back(payload->getRoadId());
                                    nodo23.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==24){
                                    renv24=renv24+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv24;
                                    nodo24.push_back(payload->getRoadId());
                                    nodo24.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==25){
                                    renv25=renv25+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv25;
                                    nodo25.push_back(payload->getRoadId());
                                    nodo25.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==26){
                                    renv26=renv26+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv26;
                                    nodo26.push_back(payload->getRoadId());
                                    nodo26.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==27){
                                    renv27=renv27+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv27;
                                    nodo27.push_back(payload->getRoadId());
                                    nodo27.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==28){
                                    renv28=renv28+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv28;
                                    nodo28.push_back(payload->getRoadId());
                                    nodo28.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==29){
                                    renv29=renv29+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv29;
                                    nodo29.push_back(payload->getRoadId());
                                    nodo29.push_back("NO");

                                   }
        if(getParentModule()->getIndex()==30){
                                    renv30=renv30+1;
                                    EV_INFO << "reenvia2 " << getParentModule()->getIndex()<< " num "<<renv30;
                                    nodo30.push_back(payload->getRoadId());
                                    nodo30.push_back("NO");
                                   }
        }
}
