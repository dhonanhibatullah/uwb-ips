import { Collection, Db, MongoClient } from 'mongodb';
import { DBConfig, CollectionType, clcTypeToString, AnchorModel, AnchorGuard, TagModel, TagGuard, NetworkModel, NetworkGuard, UserModel, UserGuard } from './config';
import logger from '../logger/logger';

class DBUtils {
    private client: MongoClient = new MongoClient(DBConfig.url);
    private database: Db = this.client.db(DBConfig.db_name);
    private valid_clc: Array<string> = DBConfig.collections;
    private tags_clc: Collection = this.database.collection(clcTypeToString(CollectionType.TAGS));
    private anchors_clc: Collection = this.database.collection(clcTypeToString(CollectionType.ANCHORS));
    private users_clc: Collection = this.database.collection(clcTypeToString(CollectionType.USERS));
    private networks_clc: Collection = this.database.collection(clcTypeToString(CollectionType.NETWORKS));
    private is_connected: boolean = false;

    private async ensureConnection(): Promise<boolean> {
        if (this.is_connected) return true;
        else {
            try {
                this.connectToDatabase();
                return true;
            } catch (error) {
                throw error;
            }
        }
    }

    public async connectToDatabase(): Promise<void> {
        try {
            await this.client.connect();
            this.is_connected = true;
            logger.info(`Successfully connected to the database [${DBConfig.url}/${DBConfig.db_name}]`);

            let avail_clc: Array<string> = (await this.database.listCollections().toArray()).map((col) => col.name);
            for (const idx in this.valid_clc) {
                if (!avail_clc.includes(this.valid_clc[idx])) {
                    await this.database.createCollection(this.valid_clc[idx]);
                }
            }
        } catch (error) {
            logger.error(`Failed to connect to the database [${DBConfig.url}/${DBConfig.db_name}]`);
            throw error;
        }
    }

    public async createAnchor(data: AnchorModel): Promise<void> {
        try {
            if (await this.ensureConnection()) {
                const result = await this.anchors_clc.insertOne(data);
                logger.info(`Create anchor success:\ndata: ${JSON.stringify(data, null, 2)}\nresult: ${JSON.stringify(result, null, 2)}`);
            }
        } catch (error) {
            logger.error(`Create anchor error: ${error}`);
            throw error;
        }
    }

    public async createTag(data: TagModel): Promise<void> {
        try {
            if (await this.ensureConnection()) {
                const result = await this.tags_clc.insertOne(data);
                logger.info(`Create tag success:\ndata: ${JSON.stringify(data, null, 2)}\nresult: ${JSON.stringify(result, null, 2)}`);
            }
        } catch (error) {
            logger.error(`Create tag error: ${error}`);
            throw error;
        }
    }

    public async createNetwork(data: NetworkModel): Promise<void> {
        try {
            if (await this.ensureConnection()) {
                const result = await this.networks_clc.insertOne(data);
                logger.info(`Create network success:\ndata: ${JSON.stringify(data, null, 2)}\nresult: ${JSON.stringify(result, null, 2)}`);
            }
        } catch (error) {
            logger.error(`Create network error: ${error}`);
            throw error;
        }
    }

    public async createUser(data: UserModel): Promise<void> {
        try {
            if (await this.ensureConnection()) {
                const result = await this.users_clc.insertOne(data);
                logger.info(`Create user success:\ndata: ${JSON.stringify(data, null, 2)}\nresult: ${JSON.stringify(result, null, 2)}`);
            }
        } catch (error) {
            logger.error(`Create user error: ${error}`);
            throw error;
        }
    }

    public async findAnchor(query: Object = {}): Promise<Array<AnchorModel>> {
        try {
            if (await this.ensureConnection()) {
                let anchor_guard: AnchorGuard = new AnchorGuard();
                let temp_data: Array<any> = (await this.anchors_clc.find(query).toArray()) as Array<any>;
                let data: Array<AnchorModel> = [];

                for (const idx in temp_data) {
                    if (anchor_guard.isAnchor(temp_data[idx])) {
                        data.push(temp_data[idx] as AnchorModel);
                    }
                }

                logger.info(`Find anchor success:\nquery: ${JSON.stringify(query, null, 2)}\ndata: ${JSON.stringify(data, null, 2)}`);

                return data;
            } else {
                return [];
            }
        } catch (error) {
            logger.error(`Find anchor error: ${error}`);
            throw error;
        }
    }

    public async findTag(query: Object = {}): Promise<Array<TagModel>> {
        try {
            if (await this.ensureConnection()) {
                let tag_guard: TagGuard = new TagGuard();
                let temp_data: Array<any> = (await this.tags_clc.find(query).toArray()) as Array<any>;
                let data: Array<TagModel> = [];

                for (const idx in temp_data) {
                    if (tag_guard.isTag(temp_data[idx])) {
                        data.push(temp_data[idx] as TagModel);
                    }
                }

                logger.info(`Find tag success:\nquery: ${JSON.stringify(query, null, 2)}\ndata: ${JSON.stringify(data, null, 2)}`);

                return data;
            } else {
                return [];
            }
        } catch (error) {
            logger.error(`Find tag error: ${error}`);
            throw error;
        }
    }

    public async findNetwork(query: Object = {}): Promise<Array<NetworkModel>> {
        try {
            if (await this.ensureConnection()) {
                let network_guard: NetworkGuard = new NetworkGuard();
                let temp_data: Array<any> = (await this.networks_clc.find(query).toArray()) as Array<any>;
                let data: Array<NetworkModel> = [];

                for (const idx in temp_data) {
                    if (network_guard.isNetwork(temp_data[idx])) {
                        data.push(temp_data[idx] as NetworkModel);
                    }
                }

                logger.info(`Find network success:\nquery: ${JSON.stringify(query, null, 2)}\ndata: ${JSON.stringify(data, null, 2)}`);

                return data;
            } else {
                return [];
            }
        } catch (error) {
            logger.error(`Find network error: ${error}`);
            throw error;
        }
    }

    public async findUser(query: Object = {}): Promise<Array<UserModel>> {
        try {
            if (await this.ensureConnection()) {
                let user_guard: UserGuard = new UserGuard();
                let temp_data: Array<any> = (await this.users_clc.find(query).toArray()) as Array<any>;
                let data: Array<UserModel> = [];

                for (const idx in temp_data) {
                    if (user_guard.isUser(temp_data[idx])) {
                        data.push(temp_data[idx] as UserModel);
                    }
                }

                logger.info(`Find user success:\nquery: ${JSON.stringify(query, null, 2)}\ndata: ${JSON.stringify(data, null, 2)}`);

                return data;
            } else {
                return [];
            }
        } catch (error) {
            logger.error(`Find user error: ${error}`);
            throw error;
        }
    }

    public async updateAnchor(query: Object, data: Object): Promise<void> {
        try {
            if (await this.ensureConnection()) {
                const result = this.anchors_clc.updateOne(query, { $set: data });
                logger.info(`Update anchor success:\nquery: ${JSON.stringify(query, null, 2)}\ndata: ${JSON.stringify(data, null, 2)}\nresult: ${JSON.stringify(result, null, 2)}`);
            }
        } catch (error) {
            logger.error(`Update anchor error: ${error}`);
            throw error;
        }
    }

    public async updateTag(query: Object, data: Object): Promise<void> {
        try {
            if (await this.ensureConnection()) {
                const result = this.tags_clc.updateOne(query, { $set: data });
                logger.info(`Update tag success:\nquery: ${JSON.stringify(query, null, 2)}\ndata: ${JSON.stringify(data, null, 2)}\nresult: ${JSON.stringify(result, null, 2)}`);
            }
        } catch (error) {
            logger.error(`Update tag error: ${error}`);
            throw error;
        }
    }

    public async updateNetwork(query: Object, data: Object): Promise<void> {
        try {
            if (await this.ensureConnection()) {
                const result = this.networks_clc.updateOne(query, { $set: data });
                logger.info(
                    `Update network success:\nquery: ${JSON.stringify(query, null, 2)}\ndata: ${JSON.stringify(data, null, 2)}\nresult: ${JSON.stringify(result, null, 2)}`
                );
            }
        } catch (error) {
            logger.error(`Update network error: ${error}`);
            throw error;
        }
    }

    public async updateUser(query: Object, data: Object): Promise<void> {
        try {
            if (await this.ensureConnection()) {
                const result = this.users_clc.updateOne(query, { $set: data });
                logger.info(`Update user success:\nquery: ${JSON.stringify(query, null, 2)}\ndata: ${JSON.stringify(data, null, 2)}\nresult: ${JSON.stringify(result, null, 2)}`);
            }
        } catch (error) {
            logger.error(`Update user error: ${error}`);
            throw error;
        }
    }

    public async deleteAnchor(query: Object): Promise<void> {}

    public async deleteTag(query: Object): Promise<void> {}

    public async deleteNetwork(query: Object): Promise<void> {}

    public async deleteUser(query: Object): Promise<void> {}
}

export { DBUtils, AnchorModel, TagModel, NetworkModel, UserModel };
