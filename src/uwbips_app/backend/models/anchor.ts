export interface AnchorModel {
    uid: string;
    node_name: string;
    uwb_id: [number, number];
    mac_address: [number, number, number, number, number, number];
    position: [number, number, number];
    is_master: boolean;
    is_active: boolean;
    is_reachable: boolean;
    calib_status: {
        time_offset: boolean;
        position: boolean;
    };
    created_at: Date;
    updated_at: Date;
}

export class AnchorGuard {
    public isAnchor(data: any): boolean {
        let anchor_key: Array<string> = [
            'uid',
            'node_name',
            'uwb_id',
            'mac_address',
            'position',
            'is_master',
            'is_active',
            'is_reachable',
            'calib_status',
            'created_at',
            'updated_at',
        ];
        let check_res: boolean = true;

        for (const idx in anchor_key) {
            if (!(anchor_key[idx] in data)) {
                check_res = false;
                break;
            }
        }

        return check_res;
    }

    public createEmptyAnchor(): AnchorModel {
        return {
            uid: '',
            node_name: '',
            uwb_id: [0, 0],
            mac_address: [0, 0, 0, 0, 0, 0],
            position: [0.0, 0.0, 0.0],
            is_master: false,
            is_active: false,
            is_reachable: false,
            calib_status: {
                time_offset: false,
                position: false,
            },
            created_at: new Date(),
            updated_at: new Date(),
        };
    }
}
