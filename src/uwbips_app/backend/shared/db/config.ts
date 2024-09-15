import { AnchorModel, AnchorGuard } from '../../models/anchor';
import { TagModel, TagGuard } from '../../models/tag';
import { NetworkModel, NetworkGuard } from '../../models/network';
import { UserModel, UserGuard } from '../../models/user';

export { AnchorModel, AnchorGuard, TagModel, TagGuard, NetworkModel, NetworkGuard, UserModel, UserGuard };

export const DBConfig = {
    url: 'mongodb://localhost:27017',
    db_name: 'uwbips-db',
    collections: ['tags', 'anchors', 'users', 'networks'],
    username: 'alamak',
    password: 'alamak',
};

export enum CollectionType {
    TAGS,
    ANCHORS,
    USERS,
    NETWORKS,
}

export function clcTypeToString(colltype: CollectionType): string {
    if (colltype == CollectionType.ANCHORS) return 'anchors';
    else if (colltype === CollectionType.TAGS) return 'tags';
    else if (colltype == CollectionType.NETWORKS) return 'networks';
    else if (colltype == CollectionType.USERS) return 'users';
    else return 'others';
}
