import inet.environment.common.PhysicalEnvironment;
import inet.mobility.single.VehicleMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.ManetRouter;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
//import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;
//import inet.physicallayer.apskradio.packetlevel.ApskScalarRadio;


network VanetRouting
{
    parameters:
        @display("bgb=1462,816;bgi=background/mapa2");
        @statistic[numRcvdPk](source=count(source.app[0].pingRxSeq); record=figure; targetFigure=numRcvdPkCounter; checkSignals=false);
        @figure[numRcvdPkCounter](type=counter; transform=scale(0.5); pos=50,300; label="Received ping replies"; decimalPlaces=4);
        @statistic[numLost](source=count(source.app[0].numLost); record=figure; targetFigure=numLostPkCounter; checkSignals=false);
        @figure[numLostPkCounter](type=counter; transform=scale(0.5); pos=50,400; label="Pings lost"; decimalPlaces=4);

    types:
        simple Unnamed
        {
        }

    submodules:
        source: ManetRouter {
            @display("p=1014,515;i=misc/aaa");
        }
        node1: ManetRouter {
            @display("p=569,622;i=misc/aaa");
        }
        node2: ManetRouter {
            @display("p=756,675;i=misc/aaa");
        }
        node3: ManetRouter {
            @display("p=847,655;i=misc/aaa");
        }
        node4: ManetRouter {
            @display("p=1021,682;i=misc/aaa");
        }
        node16: ManetRouter {
            @display("p=1223,655;i=misc/aaa");
        }
        destination: ManetRouter {
            @display("p=439.80002,439.80002;i=misc/aaa");
        }
        //ApskScalarRadioMedium
        radioMedium: Ieee80211ScalarRadioMedium {
            //radioMedium: ApskScalarRadioMedium{
            @display("p=38,28");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=194.24501,49.4775");
        }
        visualizer: IntegratedMultiVisualizer {
            @display("p=194.24501,111.7825");
        }
        //ApskScalarRadio or UnitDiskRadioMedium
        radioMedium2: <default("UnitDiskRadioMedium")> like IRadioMedium {
            //radioMedium2: ApskScalarRadio{
            @display("p=632,36");
        }
        physicalEnvironment: PhysicalEnvironment {
            @display("p=632,500");
        }
        node15: ManetRouter {
            @display("p=1143,460;i=misc/aaa");
        }
        node14: ManetRouter {
            @display("p=1165,412;i=misc/aaa");
        }
        node13: ManetRouter {
            @display("p=1267,560;i=misc/aaa");
        }
        node12: ManetRouter {
            @display("p=1232,425;i=misc/aaa");
        }
        node11: ManetRouter {
            @display("p=903.42255,546.085;i=misc/aaa");
        }
        node10: ManetRouter {
            @display("p=637.71,586.4;i=misc/aaa");
        }
        node9: ManetRouter {
            @display("p=439.80002,558.91254;i=misc/aaa");
        }
        node8: ManetRouter {
            @display("p=146,490;i=misc/aaa");
        }
        node7: ManetRouter {
            @display("p=114,498;i=misc/aaa");
        }
        node6: ManetRouter {
            @display("p=1330,701;i=misc/aaa");
        }
        node5: ManetRouter {
            @display("p=1182,686;i=misc/aaa");
        }
}
