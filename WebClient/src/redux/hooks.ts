import { TypedUseSelectorHook, useDispatch, useSelector } from 'react-redux';
import type { RootStateT, AppDispatchT } from './store';

// Use throughout your app instead of plain `useDispatch` and `useSelector`
export const useAppDispatch = () => useDispatch<AppDispatchT>();
export const useAppSelector: TypedUseSelectorHook<RootStateT> = useSelector;
