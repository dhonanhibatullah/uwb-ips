import { Db } from 'mongodb';
import { DBUtils, AnchorModel, TagModel, NetworkModel, UserModel } from '../shared/db/utils';

let db: DBUtils = new DBUtils();

db.connectToDatabase().then(() => {
    db.createTag({
        uid: 'test',
        node_name: 'test',
        uwb_id: [0, 0],
        mac_address: [0, 0, 0, 0, 0, 0],
        position: [0.0, 0.0, 0.0],
        pos_mode: 0,
        is_active: false,
        is_reachable: false,
        access_hash: [],
        created_at: new Date(),
        updated_at: new Date(),
    });
});
