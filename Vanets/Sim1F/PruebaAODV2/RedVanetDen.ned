import inet.environment.common.PhysicalEnvironment;
import inet.mobility.single.VehicleMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.ManetRouter;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;


network VanetRoutingDensidad
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
            @display("p=940.0725,514.9325;i=misc/aaa");
        }
        destination: ManetRouter {
            @display("p=440,434;i=misc/aaa");
        }
        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=38,28");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=38.461544,66.863914");
        }
        visualizer: IntegratedMultiVisualizer {
            @display("p=38,105");
        }
        radioMedium2: <default("UnitDiskRadioMedium")> like IRadioMedium {
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
            @display("p=912,542;i=misc/aaa");
        }
        node10: ManetRouter {
            @display("p=733,597.395;i=misc/aaa");
        }
        node17: ManetRouter {
            @display("p=421.475,529.5925;i=misc/aaa");
        }
        node18: ManetRouter {
            @display("p=364.6675,558.91254;i=misc/aaa");
        }
        node19: ManetRouter {
            @display("p=511.26752,586.4;i=misc/aaa");
        }
        node20: ManetRouter {
            @display("p=591.8975,558.91254;i=misc/aaa");
        }
        node9: ManetRouter {
            @display("p=440,576;i=misc/aaa");
        }
        node8: ManetRouter {
            @display("p=197.91,498.44;i=misc/aaa");
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
        node21: ManetRouter {
            @display("p=421.475,632.2125;i=misc/aaa");
        }
        node22: ManetRouter {
            @display("p=630.38,654.2025;i=misc/aaa");
        }
}
