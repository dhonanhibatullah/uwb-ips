import { AnchorModel, AnchorGuard } from '../../models/anchor';
import { TagModel, TagGuard } from '../../models/tag';
import { NetworkModel, NetworkGuard } from '../../models/network';
import { UserModel, UserGuard } from '../../models/user';
import { StateModel } from '../../models/state';

export { AnchorModel, AnchorGuard, TagModel, TagGuard, NetworkModel, NetworkGuard, UserModel, UserGuard, StateModel };

export const DBConfig = {
    url: 'mongodb://localhost:27017',
    db_name: 'uwbips-db',
    collections: ['tags', 'anchors', 'users', 'networks', 'states'],
    state_id: 'uwbips_state',
    username: 'None',
    password: 'None',
};

export enum CollectionType {
    TAGS,
    ANCHORS,
    USERS,
    NETWORKS,
    STATES,
}

export function clcTypeToString(colltype: CollectionType): string {
    if (colltype == CollectionType.ANCHORS) return 'anchors';
    else if (colltype === CollectionType.TAGS) return 'tags';
    else if (colltype == CollectionType.NETWORKS) return 'networks';
    else if (colltype == CollectionType.USERS) return 'users';
    else if (colltype == CollectionType.STATES) return 'states';
    else return 'others';
}
