import { Db } from 'mongodb';
import { DBUtils, AnchorModel, TagModel, NetworkModel, UserModel } from '../shared/db/utils';

let db: DBUtils = new DBUtils();

db.connectToDatabase().then(async () => {
    // let tag1: TagModel = await db.makeTagInstNamed();
    // await db.createTag(tag1);
    // let tag2: TagModel = await db.makeTagInstNamed();
    // await db.createTag(tag2);
    // let anc1: AnchorModel = await db.makeAnchorInstNamed();
    // await db.createAnchor(anc1);
    // let anc2: AnchorModel = await db.makeAnchorInstNamed();
    // await db.createAnchor(anc2);
    // await db.deleteTag({ uid: 'tag_1' });
    // await db.deleteTag({ uid: 'tag_2' });
    // await db.deleteAnchor({ uid: 'anchor_1' });
    // await db.deleteAnchor({ uid: 'anchor_2' });
});
