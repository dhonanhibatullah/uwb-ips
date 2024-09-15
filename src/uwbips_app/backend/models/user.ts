export interface UserModel {
    username: string;
    name: string;
    email: string;
    pass_hash: string;
    role: string;
    access_key: string;
    created_at: Date;
    updated_at: Date;
}

export class UserGuard {
    public isUser(data: any): boolean {
        let user_key: Array<string> = ['username', 'name', 'email', 'pass_hash', 'role', 'access_key', 'created_at', 'updated_at'];
        let check_res: boolean = true;

        for (const idx in user_key) {
            if (!(user_key[idx] in data)) {
                check_res = false;
                break;
            }
        }

        return check_res;
    }

    public createEmptyUser(): UserModel {
        return {
            username: '',
            name: '',
            email: '',
            pass_hash: '',
            role: '',
            access_key: '',
            created_at: new Date(),
            updated_at: new Date(),
        };
    }
}
