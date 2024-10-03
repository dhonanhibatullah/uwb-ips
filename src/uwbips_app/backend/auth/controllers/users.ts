import { UserModel, UserCreationForm, UserLoginForm, UserInfo } from '../models/users';
import { UserService, UserServiceReturnType } from '../services/users';
import { Request, Response } from 'express';

type UserCreationRequest = Request<{}, {}, UserCreationForm>;
type UserLoginRequest = Request<{}, {}, UserLoginForm>;

class UserController {
    private user_service: UserService = new UserService();

    public async login(req: UserLoginRequest, res: Response) {
        const login_form: UserLoginForm = req.body;
    }

    public async logout(req: Request, res: Response) {}

    public async getAllUsers(req: Request, res: Response) {}

    public async getUserByUsername(req: Request, res: Response) {}

    public async getUsersByDisplayName(req: Request, res: Response) {}

    public async createUser(req: Request, res: Response) {}

    public async updateUser(req: Request, res: Response) {}

    public async deleteUser(req: Request, res: Response) {}
}
