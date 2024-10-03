import * as mdb from 'mongodb';
import dotenv from 'dotenv';
dotenv.config();

export class Database {
    private client: mdb.MongoClient = new mdb.MongoClient(process.env.DB_HOST as string);
    private database: mdb.Db = this.client.db(process.env.DB_NAME as string);
    private is_connected: boolean = false;

    private async ensureConnection() {
        if (!this.is_connected) {
            await this.client.connect();
            const valid_collection: Array<string> = ['users', 'anchors', 'tags', 'networks', 'misc'];
            const avail_collection: Array<string> = (await this.database.listCollections().toArray()).map((col) => col.name);
            for (const idx in valid_collection) {
                if (!avail_collection.includes(valid_collection[idx])) {
                    await this.database.createCollection(valid_collection[idx]);
                }
            }
            this.is_connected = true;
        }
    }

    public async create(data: object, collection: 'users' | 'anchors' | 'tags' | 'networks' | 'misc') {
        try {
            await this.ensureConnection();
            return await this.database.collection(collection).insertOne(data);
        } catch (error) {
            throw error;
        }
    }

    public async find(query: object, collection: 'users' | 'anchors' | 'tags' | 'networks' | 'misc') {
        try {
            await this.ensureConnection();
            return await this.database.collection(collection).findOne(query);
        } catch (error) {
            throw error;
        }
    }

    public async findMultiple(query: object, collection: 'users' | 'anchors' | 'tags' | 'networks' | 'misc') {
        try {
            await this.ensureConnection();
            return (await this.database.collection(collection).find(query).toArray()) as Array<any>;
        } catch (error) {
            throw error;
        }
    }

    public async update(query: object, data: object, collection: 'users' | 'anchors' | 'tags' | 'networks' | 'misc') {
        try {
            await this.ensureConnection();
            return await this.database.collection(collection).updateOne(query, { $set: data });
        } catch (error) {
            throw error;
        }
    }

    public async remove(query: object, collection: 'users' | 'anchors' | 'tags' | 'networks' | 'misc') {
        try {
            await this.ensureConnection();
            return await this.database.collection(collection).deleteOne(query);
        } catch (error) {
            throw error;
        }
    }
}
