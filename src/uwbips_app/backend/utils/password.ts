import bcrypt from 'bcrypt';

export namespace Password {
    export async function hash(password: string): Promise<string> {
        const salt_rnd: number = 10;
        let hash: string = bcrypt.hashSync(password, salt_rnd);
        return hash;
    }

    export async function compare(password: string, hash: string): Promise<boolean> {
        return bcrypt.compareSync(password, hash);
    }
}
