import * as rclnodejs from 'rclnodejs';

export namespace UWBIPSMsg {
    export type NodemanControl = rclnodejs.uwbips_utilities.msg.NodemanControl;
    export type SerialData = rclnodejs.uwbips_utilities.msg.SerialData;
    export type AnchorData = rclnodejs.uwbips_utilities.msg.AnchorData;
    export type TagData = rclnodejs.uwbips_utilities.msg.TagData;
}

export namespace UWBIPSType {
    export type NodemanControl = 'uwbips_utilities/msg/NodemanControl';
    export type SerialData = 'uwbips_utilities/msg/SerialData';
    export type AnchorData = 'uwbips_utilities/msg/AnchorData';
    export type TagData = 'uwbips_utilities/msg/TagData';
}

export namespace UWBIPSInst {
    export let NodemanControl: rclnodejs.TypeClass<keyof rclnodejs.MessagesMap> = 'uwbips_utilities/msg/NodemanControl';
    export let SerialData: rclnodejs.TypeClass<keyof rclnodejs.MessagesMap> = 'uwbips_utilities/msg/SerialData';
    export let AnchorData: rclnodejs.TypeClass<keyof rclnodejs.MessagesMap> = 'uwbips_utilities/msg/AnchorData';
    export let TagData: rclnodejs.TypeClass<keyof rclnodejs.MessagesMap> = 'uwbips_utilities/msg/TagData';
}

export const UWBIPSTopic = {
    nodeman_control: 'uwbips/nodeman/control',
    espnowser_tx: 'uwbips/espnowser/tx',
    espnowser_rx: 'uwbips/espnowser/rx',
};

export const UWBIPSNodeType = {
    ANCHOR: 'anchor',
    ANCHOR_MASTER: 'anchor_master',
    TAG_TWR: 'tag_twr',
    TAG_TDOA: 'tag_tdoa',
};

export const UWBIPSNodemanCmd = {
    CREATE_NODE: 'create',
    DESTROY_NODE: 'destroy',
};

export const NodeName: string = 'uwbips_ros2gateway';
