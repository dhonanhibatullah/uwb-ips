import * as rclnodejs from 'rclnodejs';
import { UWBIPSMsg, UWBIPSType, UWBIPSInst, UWBIPSTopic, UWBIPSNodeType, UWBIPSNodemanCmd, NodeName } from './config';

class ROS2Utils {
    private ser_tx_q: Array<string> = [];
    private ser_rx_q: Array<string> = [];

    private thread(): void {
        const node: rclnodejs.Node = new rclnodejs.Node(NodeName);
        const nodeman_control_pub: rclnodejs.Publisher<UWBIPSType.NodemanControl> = node.createPublisher(UWBIPSInst.NodemanControl, UWBIPSTopic.nodeman_control);
        const espnowser_tx_pub: rclnodejs.Publisher<UWBIPSType.SerialData> = node.createPublisher(UWBIPSInst.SerialData, UWBIPSTopic.espnowser_tx);
        const espnowser_rx_sub: rclnodejs.Subscription = node.createSubscription(UWBIPSInst.SerialData, UWBIPSTopic.espnowser_rx, this.espnowserRxSubCallback);
        const node_event_timer: rclnodejs.Timer = node.createTimer(33, this.nodeEventTimerCallback);
        node.spin();
    }

    private espnowserRxSubCallback(msg: any): void {
        if (msg instanceof Buffer) return;
        const valid_msg: UWBIPSMsg.SerialData = msg as UWBIPSMsg.SerialData;
        this.ser_rx_q.push(valid_msg.data);
    }

    private nodeEventTimerCallback(): void {}

    private createNode(node_name: string, node_type: string): void {
        let msg: UWBIPSMsg.NodemanControl = {
            name: node_name,
            type: node_type,
            command: UWBIPSNodemanCmd.CREATE_NODE,
        };
    }

    public run(): void {
        rclnodejs.init().then(this.thread);
    }
}
