import { AnchorModel } from './anchor';
import { TagModel } from './tag';

export interface NetworkModel {
    name: string;
    anchors: Array<AnchorModel>;
    tags: Array<TagModel>;
}

export class NetworkGuard {
    public isNetwork(data: any): boolean {
        let network_key: Array<string> = ['name', 'anchors', 'tags'];
        let check_res: boolean = true;

        for (const idx in network_key) {
            if (!(network_key[idx] in data)) {
                check_res = false;
                break;
            }
        }

        return check_res;
    }

    public createEmptyNetwork(): NetworkModel {
        return {
            name: '',
            anchors: [],
            tags: [],
        };
    }
}
