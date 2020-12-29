use core::cmp;

pub struct AlignedChunks<'a, T> {
    a: usize,
    v: &'a [T],
    chunk_size: usize,
}

pub trait SliceExt<T> {
    /// Similar to chunks(), but where the first chunk is `origin % page_size` or `page_size` bytes.
    fn aligned_chunks(&self, origin: usize, page_size: usize) -> AlignedChunks<T>;
}

impl<T> SliceExt<T> for &[T] {
    fn aligned_chunks(&self, origin: usize, page_size: usize) -> AlignedChunks<T> {
        AlignedChunks {
            a: origin,
            v: self,
            chunk_size: page_size,
        }
    }
}

impl<'a, T> Iterator for AlignedChunks<'a, T> {
    type Item = (usize, &'a [T]);

    fn next(&mut self) -> Option<Self::Item> {
        if self.v.is_empty() {
            None
        } else {
            let first_chunk_size = self.a % self.chunk_size;
            if first_chunk_size > 0 {
                let chunksz = cmp::min(self.v.len(), first_chunk_size);
                let a = self.a;
                let (fst, snd) = self.v.split_at(chunksz);
                self.a += first_chunk_size;
                self.v = snd;
                Some((a, fst))
            } else {
                let chunksz = cmp::min(self.v.len(), self.chunk_size);
                let a = self.a;
                let (fst, snd) = self.v.split_at(chunksz);
                self.a += self.chunk_size;
                self.v = snd;
                Some((a, fst))
            }
        }
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    #[test]
    fn origin_at_boundary() {
        let buf = (0..17).collect::<Vec<u8>>();
        let buf = buf.as_slice();
        let mut iterator = buf.aligned_chunks(8, 8);
        assert_eq!(Some((8, &buf[0..8])), iterator.next());
        assert_eq!(Some((16, &buf[8..16])), iterator.next());
        assert_eq!(Some((24, &buf[16..])), iterator.next());
        assert_eq!(None, iterator.next());
    }

    #[test]
    fn origin_in_page() {
        let buf = (0..17).collect::<Vec<u8>>();
        let buf = buf.as_slice();
        let mut iterator = buf.aligned_chunks(4, 8);
        assert_eq!(Some((4, &buf[0..4])), iterator.next());
        assert_eq!(Some((8, &buf[4..12])), iterator.next());
        assert_eq!(Some((16, &buf[12..17])), iterator.next());
        assert_eq!(None, iterator.next());
    }
}
