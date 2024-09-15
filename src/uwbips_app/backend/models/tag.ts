export interface TagModel {
    uid: string;
    node_name: string;
    uwb_id: [number, number];
    mac_address: [number, number, number, number, number, number];
    position: [number, number, number];
    pos_mode: number;
    is_active: boolean;
    is_reachable: boolean;
    access_hash: Array<string>;
    created_at: Date;
    updated_at: Date;
}

export class TagGuard {
    public isTag(data: any): boolean {
        let tag_key: Array<string> = [
            'uid',
            'node_name',
            'uwb_id',
            'mac_address',
            'position',
            'pos_mode',
            'is_active',
            'is_reachable',
            'access_hash',
            'created_at',
            'updated_at',
        ];
        let check_res: boolean = true;

        for (const idx in tag_key) {
            if (!(tag_key[idx] in data)) {
                check_res = false;
                break;
            }
        }

        return check_res;
    }

    public createEmptyTag(): TagModel {
        return {
            uid: '',
            node_name: '',
            uwb_id: [0, 0],
            mac_address: [0, 0, 0, 0, 0, 0],
            position: [0.0, 0.0, 0.0],
            pos_mode: 0,
            is_active: false,
            is_reachable: false,
            access_hash: [],
            created_at: new Date(),
            updated_at: new Date(),
        };
    }
}
