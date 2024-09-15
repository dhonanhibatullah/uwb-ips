export interface StateModel {
    id: string;
    count: {
        anchor: number;
        tag: number;
        network: number;
        user: number;
    };
    last_activated_at: Date;
}
