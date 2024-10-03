export interface UserModel {
    username: string;
    pass_hash: string;
    display_name: string;
    role: 'superuser' | 'staff' | 'user';
    email: string;
    phone_number: string;
    is_active: boolean;
    is_logged: boolean;
    created_at: Date;
    updated_at: Date;
}

export interface UserCreationForm {
    username: string;
    password: string;
    display_name: string;
    role: 'superuser' | 'staff' | 'user';
    email: string | null;
    phone_number: string | null;
}

export interface UserLoginForm {
    username: string;
    password: string;
}

export interface UserInfo {
    username: string;
    display_name: string;
    role: 'superuser' | 'staff' | 'user';
    email: string;
    phone_number: string;
    is_active: boolean;
    is_logged: boolean;
    created_at: Date;
    updated_at: Date;
}
