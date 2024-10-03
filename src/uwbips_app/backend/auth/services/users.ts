import { UserModel, UserCreationForm, UserLoginForm, UserInfo } from '../models/users';
import { Database } from '../../utils/database';
import { Password } from '../../utils/password';
import { JWT } from '../../utils/jwt';
import Logger from '../../utils/logger';

export interface UserServiceReturnType {
    err: boolean;
    msg: string;
    data: any;
}

export class UserService {
    private db: Database;

    public constructor() {
        this.db = new Database();
    }

    public async loginAuth(form: UserLoginForm): Promise<UserServiceReturnType> {
        try {
            if (await this.isUsernameExist(form.username)) {
                let user_found = (await this.db.find({ username: form.username }, 'users')) as any;
                let user_info = this.getInfoFromModel(user_found);
                let pass_comp: boolean = await Password.compare(form.password, user_found.pass_hash);
                if (pass_comp) {
                    await this.setUserLoggedStatus(form.username, true);
                    let access_tkn: string = JWT.generateAccessToken(user_info);
                    let refresh_tkn: string = JWT.generateRefreshToken(user_info);
                    return {
                        err: false,
                        msg: `Login authentication success.`,
                        data: {
                            access_token: access_tkn,
                            refresh_token: refresh_tkn,
                        },
                    };
                } else {
                    return {
                        err: true,
                        msg: `Login authentication failed: Wrong password.`,
                        data: null,
                    };
                }
            } else {
                return {
                    err: true,
                    msg: `Login authentication failed: Username [${form.username}] does not exist.`,
                    data: null,
                };
            }
        } catch (error) {
            return {
                err: true,
                msg: `Login authentication failed: ${error}`,
                data: null,
            };
        }
    }

    public async logout(username: string): Promise<UserServiceReturnType> {
        try {
            if (await this.isUsernameExist(username)) {
                await this.setUserLoggedStatus(username, false);
                return {
                    err: false,
                    msg: 'Logout success',
                    data: username,
                };
            } else {
                return {
                    err: false,
                    msg: 'Logout failed: ',
                    data: username,
                };
            }
        } catch (error) {
            return {
                err: true,
                msg: `Logout failed: ${error}`,
                data: username,
            };
        }
    }

    public async getAllUsers(): Promise<UserServiceReturnType> {
        try {
            let users_arr: Array<any> = await this.db.findMultiple({}, 'users');
            let users: Array<UserInfo> = [];
            for (const user of users_arr) users.push(this.getInfoFromModel(user));
            return {
                err: false,
                msg: 'Get all users success.',
                data: users,
            };
        } catch (error) {
            return {
                err: true,
                msg: `Get all users failed: ${error}`,
                data: null,
            };
        }
    }

    public async getUserByUsername(username: string) {
        try {
            if (await this.isUsernameExist(username)) {
                let user_found = (await this.db.find({ username: username }, 'users')) as any;
                let user: UserInfo = this.getInfoFromModel(user_found);
                return {
                    err: false,
                    msg: `Get user with [username: ${username}] success.`,
                    data: user,
                };
            } else {
                return {
                    err: true,
                    msg: 'Get user with [username: ${username}] failed: Username does not exist.',
                    data: null,
                };
            }
        } catch (error) {
            return {
                err: true,
                msg: `Get user with [username: ${username}] failed: ${error}`,
                data: null,
            };
        }
    }

    public async getUsersByDisplayName(display_name: string) {
        try {
            let users_arr: Array<any> = await this.db.findMultiple({ display_name: display_name }, 'users');
            let users: Array<UserInfo> = [];
            for (const user of users_arr) users.push(this.getInfoFromModel(user));
            return {
                err: false,
                msg: `Get users with [display_name: ${display_name}] success.`,
                data: users,
            };
        } catch (error) {
            return {
                err: true,
                msg: `Get users with [display_name: ${display_name}] failed: ${error}`,
                data: null,
            };
        }
    }

    public async createUser(form: UserCreationForm): Promise<UserServiceReturnType> {
        try {
            if (!(await this.isUsernameExist(form.username))) {
                let new_user: UserModel = await this.getModelFromForm(form);
                await this.db.create(new_user, 'users');
                return {
                    err: false,
                    msg: `User [username: ${form.username}] creation success.`,
                    data: form,
                };
            } else {
                return {
                    err: true,
                    msg: `User [username: ${form.username}] creation failed: Username is already taken.`,
                    data: null,
                };
            }
        } catch (error) {
            return {
                err: true,
                msg: `User [username: ${form.username}] creation failed: ${error}`,
                data: null,
            };
        }
    }

    public async updateUser(username: string, password?: string, display_name?: string, role?: 'superuser' | 'staff' | 'user', email?: string, phone_number?: string): Promise<UserServiceReturnType> {
        let is_updated: boolean = false;
        let update_attr = {};

        if (password !== undefined) {
            update_attr['pass_hash'] = Password.hash(password);
            is_updated = true;
        }
        if (display_name !== undefined) {
            update_attr['display_name'] = display_name;
            is_updated = true;
        }
        if (role !== undefined) {
            update_attr['role'] = role;
            is_updated = true;
        }
        if (email !== undefined) {
            update_attr['email'] = email;
            is_updated = true;
        }
        if (phone_number !== undefined) {
            update_attr['phone_number'] = phone_number;
            is_updated = true;
        }

        if (is_updated) {
            try {
                await this.db.update({ username: username }, update_attr, 'users');
                return {
                    err: false,
                    msg: `User [username: ${username}] update success.`,
                    data: update_attr,
                };
            } catch (error) {
                return {
                    err: true,
                    msg: `User [username: ${username}] update failed: ${error}`,
                    data: update_attr,
                };
            }
        } else {
            return {
                err: true,
                msg: `User [username: ${username}] update failed: No argument passed.`,
                data: null,
            };
        }
    }

    public async deleteUser(username: string): Promise<UserServiceReturnType> {
        try {
            if (await this.isUsernameExist(username)) {
                await this.db.remove({ username: username }, 'users');
                return {
                    err: false,
                    msg: `User [username: ${username}] deletion success.`,
                    data: null,
                };
            } else {
                return {
                    err: true,
                    msg: `User [username: ${username}] deletion failed: Username does not exist.`,
                    data: null,
                };
            }
        } catch (error) {
            return {
                err: true,
                msg: `User [username: ${username}] deletion failed: ${error}`,
                data: null,
            };
        }
    }

    private async isUsernameExist(username: string): Promise<boolean> {
        try {
            let res = await this.db.find({ username: username }, 'users');
            if (res == null) return false;
            return true;
        } catch (error) {
            throw error;
        }
    }

    private async getModelFromForm(form: UserCreationForm): Promise<UserModel> {
        return {
            username: form.username,
            pass_hash: await Password.hash(form.password),
            display_name: form.display_name,
            role: form.role,
            email: form.email as string,
            phone_number: form.phone_number as string,
            is_active: true,
            is_logged: false,
            created_at: new Date(),
            updated_at: new Date(),
        };
    }

    private getInfoFromModel(user_data: any): UserInfo {
        return {
            username: user_data.username,
            display_name: user_data.display_name,
            role: user_data.role,
            email: user_data.email,
            phone_number: user_data.phone_number,
            is_active: user_data.is_active,
            is_logged: user_data.is_logged,
            created_at: user_data.created_at,
            updated_at: user_data.updated_at,
        };
    }

    private async setUserLoggedStatus(username: string, log_sts: boolean): Promise<void> {
        try {
            await this.db.update({ username: username }, { is_logged: log_sts }, 'users');
        } catch (error) {
            throw error;
        }
    }
}
