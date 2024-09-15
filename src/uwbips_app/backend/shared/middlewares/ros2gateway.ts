import * as rclnodejs from 'rclnodejs';

type NodemanControlMsg = rclnodejs.uwbips_utilities.msg.NodemanControl;
type SerialDataMsg = rclnodejs.uwbips_utilities.msg.SerialData;
type TagDataMsg = rclnodejs.uwbips_utilities.msg.TagData;
type AnchorDataMsg = rclnodejs.uwbips_utilities.msg.AnchorData;

interface uwbips_topic {
    nodeman_control: string;
    espnowser_tx: string;
    espnowser_rx: string;
}

class ROS2Gateway {
    private topic: uwbips_topic = {
        nodeman_control: 'uwbips/nodeman/control',
        espnowser_tx: 'uwbips/espnowser/tx',
        espnowser_rx: 'uwbips/espnowser/rx',
    };
    private node_name: string = '';
    private ser_tx_q: Array<string> = [];
    private ser_rx_q: Array<string> = [];

    private thread(): void {
        const node: rclnodejs.Node = new rclnodejs.Node(this.node_name);

        const nodeman_control_pub: rclnodejs.Publisher<'uwbips_utilities/msg/NodemanControl'> =
            node.createPublisher(
                'uwbips_utilities/msg/NodemanControl',
                this.topic.nodeman_control
            );

        const espnowser_tx_pub: rclnodejs.Publisher<'uwbips_utilities/msg/SerialData'> =
            node.createPublisher(
                'uwbips_utilities/msg/SerialData',
                this.topic.espnowser_tx
            );

        const espnowser_rx_sub: rclnodejs.Subscription =
            node.createSubscription(
                'uwbips_utilities/msg/SerialData',
                this.topic.espnowser_rx,
                (msg: Buffer | SerialDataMsg) => {
                    if (msg instanceof Buffer) return;
                    const valid_msg: SerialDataMsg = msg as SerialDataMsg;
                    this.ser_rx_q.push(msg.data);
                }
            );

        const espnowser_tx_timer: rclnodejs.Timer = node.createTimer(
            33,
            () => {}
        );

        node.spin();
    }

    public constructor(node_name: string) {
        this.node_name = node_name;
    }

    public run(): void {
        rclnodejs.init().then(this.thread);
    }

    public serialWrite(): boolean {
        return true;
    }
}
